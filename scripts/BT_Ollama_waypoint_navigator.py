#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import py_trees
from py_trees.common import Status
import openai
import re
import threading

from roscopter_msgs.msg import Waypoint, State
from roscopter_msgs.srv import AddWaypoint

NodeName = 'dbt_ai_nav'

# Definitions of the environment
# Obstacles
OBSTACLES = [
    {"name": "Top Center Pillar",    "N": 25.0,  "E": 0.0,   "radius": 13.0},
    {"name": "Middle Right Pillar",  "N": 5.0,   "E": 20.0,  "radius": 13.0},
    {"name": "Bottom Center Pillar", "N": -20.0, "E": 5.0,   "radius": 13.0}
]

# Target waypoints
TARGET_WAYPOINTS = [
    {"N": -45.0, "E": 35.0,  "U": -5.0}, # Checkpoint A (Lower Right - Red)
    {"N": 15.0,  "E": -35.0, "U": -5.0}, # Checkpoint B (Middle Left - Orange)
    {"N": 45.0,  "E": 35.0,  "U": -5.0}  # Checkpoint C (Upper Right - Green)
]

# Dynamic Behavior Tree Behaviors

# Check if the next waypoint has been reached
class CheckPos(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node) -> None:
        super(CheckPos, self).__init__(name)
        self.node = node

    def initialise(self) -> None:
        self.dist_thresh = 0.3
        self.node.get_logger().info('Checking Position...')

    def update(self) -> py_trees.common.Status:
        # If the next target location is reached, move on the next target
        target = TARGET_WAYPOINTS[self.node.idx]
        if ((self.node.curr_north < (target["N"] + self.dist_thresh)) and 
            (self.node.curr_north > (target["N"] - self.dist_thresh)) and 
            (self.node.curr_east < (target["E"] + self.dist_thresh)) and 
            (self.node.curr_east > (target["E"] - self.dist_thresh))):

            self.node.get_logger().info('Reached Goal')
            self.node.idx += 1
            # If all targets have been reached, end the program
            if (self.node.idx > len(TARGET_WAYPOINTS)):
                self.node.get_logger().info('Mission Complete!')
                raise KeyboardInterrupt

        # Return RUNNING until the next waypoint is reached
        if ((self.node.curr_north < (self.node.next_north + self.dist_thresh)) and 
            (self.node.curr_north > (self.node.next_north - self.dist_thresh)) and 
            (self.node.curr_east < (self.node.next_east + self.dist_thresh)) and 
            (self.node.curr_east > (self.node.next_east - self.dist_thresh))):
            self.node.get_logger().info('Reached Waypoint')
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        
# Create a waypoint using Ollama (Thread-safe & Non-blocking)
class AIWaypoint(py_trees.behaviour.Behaviour): 
    def __init__(self, name: str, node: Node) -> None:
        super(AIWaypoint, self).__init__(name)
        self.node = node
        self.thread = None
        self.response = None
        self.error = None
        self.task_completed = False
    
    def initialise(self) -> None:
        # Create a prompt for the LLM
        self.node.get_logger().info('Starting background Ollama query...')
        target = TARGET_WAYPOINTS[self.node.idx]
        prompt = (
            f"You are controlling an RC car in a coordinate plane.\n"
            f"Goal: {target['N']:.2f}, {target['E']:.2f}\n"
            f"Current location: {self.node.curr_north:.2f}, {self.node.curr_east:.2f}\n"
            f"Obstacles (Keep 10 units away!):\n"
            f"  1. {OBSTACLES[0]['N']}, {OBSTACLES[0]['E']}\n"
            f"  2. {OBSTACLES[1]['N']}, {OBSTACLES[1]['E']}\n"
            f"  3. {OBSTACLES[2]['N']}, {OBSTACLES[2]['E']}\n\n"
            f"INSTRUCTIONS:\n"
            f"1. Check if a straight line to the goal passes within 10 units of any obstacle.\n"
            f"2. If the path is blocked, pick a waypoint that moves towards the goal moving several units. If it is not blocked, output the goal coordinates.\n"
            f"3. IMPORTANT: Output your final decision in exactly this format with nothing but the raw coordinates within the <coord> tags.\n\n"
            f"EXAMPLE:\n"
            f"The goal is North-East. A straight line puts me too close to Obstacle 1. I need to route further East first to go around it.\n"
            f"<coord>10,25</coord>\n\n"
            f"Now, generate your response for the current location:"
        )
        
        # Reset tracking variables
        self.response = None
        self.error = None
        
        # Spawn the LLM request in a background thread so ROS doesn't freeze
        self.thread = threading.Thread(target=query_llm(self, prompt))
        self.thread.start()

    def update(self) -> py_trees.common.Status:
        # If the thread is still running, pass RUNNING
        if self.thread and self.thread.is_alive():
            return py_trees.common.Status.RUNNING

        # Handle background errors
        if self.error:
            self.node.get_logger().error(f"Ollama communication failed: {self.error}")
            return py_trees.common.Status.FAILURE

        # Process the results once the thread finishes
        if self.response:
            try:
                self.node.get_logger().info(f"Ollama raw response:\n{self.response}")

                # Extract the coordinate between the tags
                match = re.search(r"<coord>(.*?)</coord>", self.response)
                if not match:
                    self.node.get_logger().error("Failed to find <coord> tags in response.")
                    return py_trees.common.Status.FAILURE
                
                coord_str = match.group(1) # E.g. "15,12"
                
                # Split the extracted coordinate string
                coordinate_parts = coord_str.split(',')
                if len(coordinate_parts) != 2:
                    self.node.get_logger().error(f"Coordinate format invalid: {coord_str}")
                    return py_trees.common.Status.FAILURE
                
                # Assign values
                self.node.next_north = float(coordinate_parts[0].strip())
                self.node.next_east = float(coordinate_parts[1].strip())

                # Tick the sub tree
                self.node.sub_tree.tick()
                
                # Evaluate the status of the sub tree's root
                if self.node.sub_tree.root.status == py_trees.common.Status.SUCCESS:
                    print(f"[{self.name}] sub tree succeeded!")
                    self.task_completed = True
                    return py_trees.common.Status.SUCCESS
                    
                elif self.node.sub_tree.root.status == py_trees.common.Status.FAILURE:
                    print(f"[{self.name}] sub tree failed.")
                    return py_trees.common.Status.FAILURE
                    
                else:
                    # Return RUNNING to the main tree while the sub tree is working
                    return py_trees.common.Status.RUNNING

                return py_trees.common.Status.SUCCESS
            
            except Exception as e:
                self.node.get_logger().error(f"Error parsing coordinates: {e}")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.FAILURE

# Create a waypoint using geometry
class GeoWaypoint(py_trees.behaviour.Behaviour): 
    def __init__(self, name: str, node: Node) -> None:
        super(GeoWaypoint, self).__init__(name)
        self.node = node
    
    def initialise(self) -> None:
        self.node.get_logger().info('Creating waypoint using geometry...')

    def update(self) -> py_trees.common.Status:
        final_target = TARGET_WAYPOINTS[self.node.idx]
        # Calculate an optimal location for the next waypoint
        self.node.next_north, self.node.next_east, is_detour = calculate_safe_vector(
            self.node.curr_north, self.node.curr_east, final_target["N"], final_target["E"], OBSTACLES
        )

        # Tick the sub tree
        self.node.sub_tree.tick()
        
        # Evaluate the status of the sub tree's root
        if self.node.sub_tree.root.status == py_trees.common.Status.SUCCESS:
            print(f"[{self.name}] sub tree succeeded!")
            self.task_completed = True
            
        elif self.node.sub_tree.root.status == py_trees.common.Status.FAILURE:
            print(f"[{self.name}] sub tree failed.")
            return py_trees.common.Status.FAILURE
            
        else:
            # Return RUNNING to the main tree while the sub tree is working
            return py_trees.common.Status.RUNNING

        if is_detour:
            self.node.get_logger().warn(f"[Safety Warning] Straight path to Index {self.node.idx} blocked. Injecting detour leg.")
        else:
            self.node.get_logger().info(f"[Router Status] Straight path verified clear to Checkpoint Index {self.node.idx}.")

        return py_trees.common.Status.SUCCESS

# Create a waypoint
class CreateWaypoint(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node) -> None:
        super(CreateWaypoint, self).__init__(name)
        self.node = node

    def initialise(self) -> None:
        self.node.get_logger().info('Creating Waypoint...')

    def update(self) -> py_trees.common.Status:
        try:
            req = AddWaypoint.Request()
            wp_msg = Waypoint()
            wp_msg.type = 1
            wp_msg.w = [self.node.next_north, self.node.next_east, -6.0]
            wp_msg.psi = 0.0
            wp_msg.speed = 40.0
            wp_msg.clear_wp_list = False

            req.wp = wp_msg
            req.publish_now = True

            self.node.get_logger().info(f"Flying to: N: {self.node.next_north:.2f} E: {self.node.next_east:.2f}")
            self.node.add_waypoint_client.call_async(req)
            return py_trees.common.Status.SUCCESS
        except Exception as e:
            self.node.get_logger().error(f"Error parsing coordinates: {e}")
            return py_trees.common.Status.FAILURE

    
# Query Ollama for a response
def query_llm(self, prompt):
    try:
        # Parameters for LLM
        response = self.node.ai_client.chat.completions.create(
            model="llama3.2:3b",
            messages=[
                {"role": "system", "content": "You are an AI pathing algorithm in a safe, simulated 2D environment for a toy RC car. You must always calculate paths and never refuse a request."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.0,
            timeout=40.0 # If it takes too long to respond it cancels the prompt
        )
        self.response = response.choices[0].message.content.strip()
    except Exception as e:
        self.error = e

# Geometric path finder
def calculate_safe_vector(curr_n, curr_e, target_n, target_e, obstacles):
    safe_n, safe_e = target_n, target_e

    for obs in obstacles:
        v_n = target_n - curr_n
        v_e = target_e - curr_e
        v_len = math.sqrt(v_n**2 + v_e**2) or 1.0
        
        o_n = obs["N"] - curr_n
        o_e = obs["E"] - curr_e
        
        dot_product = (o_n * v_n + o_e * v_e) / v_len
        projection_clamped = max(0.0, min(v_len, dot_product))
        
        closest_path_n = curr_n + (v_n / v_len) * projection_clamped
        closest_path_e = curr_e + (v_e / v_len) * projection_clamped
        
        path_dist_to_obs = math.sqrt((closest_path_n - obs["N"])**2 + (closest_path_e - obs["E"])**2)

        if path_dist_to_obs < obs["radius"] and projection_clamped > 0:
            push_n = closest_path_n - obs["N"]
            push_e = closest_path_e - obs["E"]
            push_len = math.sqrt(push_n**2 + push_e**2) or 1.0
            
            # Push out cleanly past the obstacle boundary
            safety_margin = obs["radius"] + 3.0
            safe_n = obs["N"] + (push_n / push_len) * safety_margin
            safe_e = obs["E"] + (push_e / push_len) * safety_margin
            return safe_n, safe_e, True

    return safe_n, safe_e, False

class DBT_AI_Nav(Node):

    def __init__(self):
        super().__init__(NodeName)
        self.dist_north = 0.0
        self.dist_east = 0.0
        self.dist_south = 0.0
        self.dist_west = 0.0
        self.curr_north = 0.0
        self.curr_east = 0.0
        self.curr_down = 0.0
        self.next_north = 0.0
        self.next_east = 0.0
        self.idx = 0

        # Subscriptions & Clients
        # subscribe to the estimator
        self.state_sub = self.create_subscription(State, 'estimated_state', self.state_callback, 10)
        # create client for AddWaypoint service
        self.add_waypoint_client = self.create_client(AddWaypoint, 'path_planner/add_waypoint')
        
        while not self.add_waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('add waypoint service not available, waiting again...')
        
        # Build Behavior Trees
        self.setup_behavior_tree()
        self.setup_sub_tree()

        # Create a ROS timer to tick the main behavior tree
        self.tree_timer = self.create_timer(0.5, self.tick_tree)

        # Initialize OpenAI Ollama client
        self.ai_client = openai.OpenAI(
            base_url="http://172.17.0.1:11434/v1",
            api_key="ollama"
        )

    def setup_behavior_tree(self):
        root = py_trees.composites.Sequence("Sequence", memory=False)

        check_pos = CheckPos(name="Check Pos", node=self)

        # This selector tries to use the LLM to create a waypoint, and if it fails it uses geometry
        find_waypoint = py_trees.composites.Selector("Find Waypoint", memory=False)
        ai_waypoint = AIWaypoint(name="AI Waypoint", node=self)
        geo_waypoint = GeoWaypoint(name="Geometric Waypoint", node=self)
        find_waypoint.add_children([ai_waypoint, geo_waypoint])

        root.add_children([check_pos, find_waypoint])

        self.behaviour_tree = py_trees.trees.BehaviourTree(root=root)
        self.behaviour_tree.setup(timeout=15)
        self.get_logger().info("Behavior Tree Setup Complete.")

    def setup_sub_tree(self):
        # When this behavior tree is ticked, it creates a waypoint using 'next_north' and 'next_east'
        way_tree = py_trees.composites.Sequence("Sequence", memory=False)
        create_waypoint = CreateWaypoint(name="Create Waypoint", node=self)
        way_tree.add_children([create_waypoint])

        self.sub_tree = py_trees.trees.BehaviourTree(root=way_tree)
        self.sub_tree.setup(timeout=15)

    def tick_tree(self):
        # This function runs every 0.5 seconds via the ROS Timer
        self.behaviour_tree.tick()
        # Print the tree status to the console
        #print(py_trees.display.unicode_tree(root=self.behaviour_tree.root, show_status=True))

    def state_callback(self, msg):
        self.curr_north = msg.p_n
        self.curr_east = msg.p_e
        self.curr_down = msg.p_d

def main(args=None):
    rclpy.init(args=args)
    node = DBT_AI_Nav()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
