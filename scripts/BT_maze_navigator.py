#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
import py_trees
from py_trees.common import Status
import openai

from std_msgs.msg import Float32MultiArray
from roscopter_msgs.msg import Waypoint, State
from roscopter_msgs.srv import AddWaypoint

# STEP 1: DYNAMIC WAYPOINT ACTION NODE (With Position Tracking)
class DynamicWaypointAction(py_trees.behaviour.Behaviour):
    """
    Action node constructed at runtime by the router. 
    Sends flight targets to the autopilot and remains RUNNING 
    until the drone arrives within a 1-meter threshold.
    """
    def __init__(self, name, target_n, target_e, speed, node):
        super().__init__(name)
        self.target_n = target_n
        self.target_e = target_e
        self.speed = speed
        self.node = node
        self.cmd_sent = False
        
        # Attach blackboard client to monitor current telemetry
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="curr_n", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="curr_e", access=py_trees.common.Access.READ)

    def update(self):
        # 1. Fire the waypoint service command once
        if not self.cmd_sent:
            req = AddWaypoint.Request()
            wp_msg = Waypoint()
            wp_msg.type = 1  
            wp_msg.w = [self.target_n, self.target_e, -6.0]  
            wp_msg.psi = 0.0         
            wp_msg.speed = self.speed  
            wp_msg.clear_wp_list = False  
            
            req.wp = wp_msg
            req.publish_now = True
            
            self.node.waypoint_client.call_async(req)
            self.node.get_logger().info(f"Target sent - N: {self.target_n:.2f}, E: {self.target_e:.2f}, Speed: {self.speed}")
            self.cmd_sent = True
            return Status.RUNNING

        # 2. Check current progress
        curr_n = self.blackboard.get("curr_n")
        curr_e = self.blackboard.get("curr_e")
        
        distance_to_target = math.sqrt((self.target_n - curr_n)**2 + (self.target_e - curr_e)**2)

        # 3. Hold focus until we finish moving down the turn corridor
        if distance_to_target > 1.0:
            return Status.RUNNING
            
        self.node.get_logger().info("Arrived at dynamic turn waypoint.")
        return Status.SUCCESS

# STEP 2: MUTATOR NODE
class DynamicMazeRouter(py_trees.behaviour.Behaviour):
    """
    Monitors telemetry. When a wall is approached, it queries Ollama,
    removes itself, and grafts a new branch into the tree.
    """
    def __init__(self, name="DynamicMazeRouter", node=None):
        super().__init__(name)
        self.node = node  
        self.blackboard = self.attach_blackboard_client(name=self.name)
        
        self.blackboard.register_key(key="flight_history", access=py_trees.common.Access.WRITE)
        
        self.blackboard.register_key(key="d_north", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="d_east", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="d_south", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="d_west", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="curr_n", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="curr_e", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_axis", access=py_trees.common.Access.READ)
        
        self.blackboard.register_key(key="target_n", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_e", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_axis", access=py_trees.common.Access.WRITE)

        self.ai_client = openai.OpenAI(
            base_url="http://172.17.0.1:11434/v1",
            api_key="ollama"
        )

    def update(self):
        dn = self.blackboard.get("d_north")
        de = self.blackboard.get("d_east")
        ds = self.blackboard.get("d_south")
        dw = self.blackboard.get("d_west")
        curr_axis = self.blackboard.get("current_axis")

        if dn == 0.0 and de == 0.0 and ds == 0.0 and dw == 0.0:
            return Status.RUNNING

        hit_wall = False
        if curr_axis == "north" and dn < 4.2: hit_wall = True
        elif curr_axis == "east" and de < 4.2: hit_wall = True
        elif curr_axis == "south" and ds < 4.2: hit_wall = True
        elif curr_axis == "west" and dw < 4.2: hit_wall = True

        if hit_wall:
            self.node.get_logger().warn(f"Wall detected on axis: {curr_axis}")
            
            history = self.blackboard.get("flight_history")
            history_str = " -> ".join(history) if history else "None (Just launched)"

            prompt = (
                f"You are an autonomous drone navigation agent flying inside a maze hallway.\n"
                f"Your recent directional history is: {history_str}\n\n"
                f"You have arrived at an intersection. Your sensors report the following distances to the walls:\n"
                f"- North: {dn:.2f} meters\n"
                f"- East: {de:.2f} meters\n"
                f"- South: {ds:.2f} meters\n"
                f"- West: {dw:.2f} meters\n\n"
                f"Analyze these readings. Identify which direction is an open hallway rather than a dead-end.\n"
                f"CRITICAL: Avoid choosing a direction that immediately backtracks or cycles through your recent history "
                f"unless all other options are completely blocked.\n\n"
                f"Reply with exactly one word from this list: north, east, south, or west. Do not include any other text or punctuation."
            )

            try:
                self.node.get_logger().info("Querying Llama3...")
                response = self.ai_client.chat.completions.create(
                    model="llama3.2:1b",
                    messages=[{"role": "user", "content": prompt}],
                    temperature=0.0  
                )
                
                ai_decision = response.choices[0].message.content.strip().lower()
                ai_decision = "".join(c for c in ai_decision if c.isalpha())
                self.node.get_logger().info(f"AI Decision: {ai_decision}")
                
                history.append(ai_decision)
                if len(history) > 4:
                    history.pop(0)
                self.blackboard.set("flight_history", history)
                
            except Exception as e:
                self.node.get_logger().error(f"Ollama communication failed: {e}")
                return Status.FAILURE

            curr_n = self.blackboard.get("curr_n")
            curr_e = self.blackboard.get("curr_e")
            BUFFER = 4.0

            if ai_decision == "north":
                new_n = curr_n + dn - BUFFER
                new_e = curr_e
            elif ai_decision == "south":
                new_n = curr_n - ds + BUFFER
                new_e = curr_e
            elif ai_decision == "east":
                new_n = curr_n
                new_e = curr_e + de - BUFFER
            elif ai_decision == "west":
                new_n = curr_n
                new_e = curr_e - dw + BUFFER
            else:
                self.node.get_logger().warn(f"Invalid AI output received: {ai_decision}")
                return Status.FAILURE

            self.blackboard.set("target_n", new_n)
            self.blackboard.set("target_e", new_e)
            self.blackboard.set("current_axis", ai_decision)

            # Reconstruct tree branch
            parent_composite = self.parent
            cautious_turn_branch = py_trees.composites.Sequence(name="Dynamic Turn Branch", memory=False)
            
            dynamic_action_node = DynamicWaypointAction(
                name=f"Turn_{ai_decision.upper()}",
                target_n=new_n,
                target_e=new_e,
                speed=12.0,  
                node=self.node
            )
            
            cautious_turn_branch.add_child(dynamic_action_node)
            
            self.node.get_logger().info("Mutating tree structure...")
            parent_composite.replace_child(self, cautious_turn_branch)
            
            self.node.reset_router_flag = True
            return Status.SUCCESS  

        return Status.RUNNING  

# STEP 3: MASTER CONTROLLER NODE
class BTNavMaster(Node):
    def __init__(self):
        super().__init__('bt_maze_navigator')
        
        self.wall_sub = self.create_subscription(Float32MultiArray, 'sensors/walls_sensor', self.wall_cb, 10)
        self.state_sub = self.create_subscription(State, 'estimated_state', self.state_cb, 10)
        self.waypoint_client = self.create_client(AddWaypoint, '/path_planner/add_waypoint')
        
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.set("d_north", 0.0)
        self.blackboard.set("d_east", 0.0)
        self.blackboard.set("d_south", 0.0)
        self.blackboard.set("d_west", 0.0)
        self.blackboard.set("curr_n", 0.0)
        self.blackboard.set("curr_e", 0.0)
        self.blackboard.set("target_n", 0.0)
        self.blackboard.set("target_e", 0.0)
        self.blackboard.set("current_axis", "east")
        self.blackboard.set("flight_history", [])
        
        self.takeoff_initiated = False
        self.reset_router_flag = False

        self.root = py_trees.composites.Sequence(name="Dynamic Maze Master Root", memory=False)
        self.active_router = DynamicMazeRouter(name="Baseline Routing Engine", node=self)
        self.root.add_child(self.active_router)
        
        self.tree_manager = py_trees.trees.BehaviourTree(self.root)
        self.tree_manager.setup(timeout=15.0)

        self.create_timer(0.1, self.loop_callback)
        self.get_logger().info("Node initialized.")

    def wall_cb(self, msg):
        self.blackboard.set("d_north", msg.data[0])
        self.blackboard.set("d_east", msg.data[1])
        self.blackboard.set("d_south", msg.data[2])
        self.blackboard.set("d_west", msg.data[3])

    def state_cb(self, msg):
        self.blackboard.set("curr_n", msg.p_n)
        self.blackboard.set("curr_e", msg.p_e)

    def loop_callback(self):
        if not self.takeoff_initiated and self.blackboard.get("d_north") > 0.0:
            self.takeoff_initiated = True
            
            req = AddWaypoint.Request()
            wp_msg = Waypoint()
            wp_msg.type = 1
            wp_msg.w = [self.blackboard.get("curr_n") + self.blackboard.get("d_north") - 4.0, self.blackboard.get("curr_e"), -6.0]
            wp_msg.psi = 0.0
            wp_msg.speed = 25.0
            wp_msg.clear_wp_list = False
            req.wp = wp_msg
            req.publish_now = True
            
            self.blackboard.set("target_n", wp_msg.w[0])
            self.blackboard.set("target_e", wp_msg.w[1])
            self.blackboard.set("current_axis", "north")
            
            self.waypoint_client.call_async(req)
            self.get_logger().info("Dispatched initial launch waypoint.")
            return

        if self.takeoff_initiated:
            # Check if active dynamic branch successfully completed its travel
            if self.reset_router_flag and self.root.children[0].status == Status.SUCCESS:
                self.root.remove_all_children()
                self.active_router = DynamicMazeRouter(name="Baseline Routing Engine", node=self)
                self.root.add_child(self.active_router)
                self.reset_router_flag = False
                self.get_logger().info("Tree structure safely reset to baseline router after arrival.")

            self.tree_manager.tick()

def main(args=None):
    rclpy.init(args=args)
    node = BTNavMaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
