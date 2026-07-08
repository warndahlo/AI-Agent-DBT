#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import py_trees
from py_trees.common import Status

from roscopter_msgs.msg import Waypoint, State
from roscopter_msgs.srv import AddWaypoint

# ==============================================================================
# DEFINITIONS OF THE MAZE ENVIRONMENT
# ==============================================================================
# Pillars tracked relative to the red (+North) axis pointing up
OBSTACLES = [
    {"name": "Top Center Pillar",    "N": 25.0,  "E": 0.0,   "radius": 13.0},
    {"name": "Middle Right Pillar",  "N": 5.0,   "E": 20.0,  "radius": 13.0},
    {"name": "Bottom Center Pillar", "N": -20.0, "E": 5.0,   "radius": 13.0}
]

# Checkpoint coordinates mapped directly from the screen visual layout
TARGET_WAYPOINTS = [
    {"N": -45.0, "E": 35.0,  "U": -5.0}, # Checkpoint A (Lower Right - Red)
    {"N": 15.0,  "E": -35.0, "U": -5.0}, # Checkpoint B (Middle Left - Orange)
    {"N": 45.0,  "E": 35.0,  "U": -5.0}  # Checkpoint C (Upper Right - Green)
]

# ==============================================================================
# GEOMETRIC COLLISION AVOIDANCE ENGINE
# ==============================================================================
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
            safety_margin = obs["radius"] + 6.0
            safe_n = obs["N"] + (push_n / push_len) * safety_margin
            safe_e = obs["E"] + (push_e / push_len) * safety_margin
            return safe_n, safe_e, True

    return safe_n, safe_e, False

# ==============================================================================
# TRACKING ACTION LEAF WITH CHECKPOINT ACCOUNTABILITY
# ==============================================================================
class SafeWaypointAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_n, target_e, altitude, speed, is_detour, node):
        super().__init__(name)
        self.target_n = target_n
        self.target_e = target_e
        self.altitude = altitude  
        self.speed = speed
        self.is_detour = is_detour
        self.node = node
        self.cmd_sent = False
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="wp_index", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="curr_n", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="curr_e", access=py_trees.common.Access.READ)

    def update(self):
        if not self.cmd_sent:
            req = AddWaypoint.Request()
            wp_msg = Waypoint()
            wp_msg.type = 1  
            wp_msg.w = [self.target_n, self.target_e, self.altitude]  
            wp_msg.psi = 0.0         
            wp_msg.speed = self.speed  
            wp_msg.clear_wp_list = False  
            
            req.wp = wp_msg
            req.publish_now = True
            
            self.node.waypoint_client.call_async(req)
            self.node.get_logger().info(f"[Flight Cmd] Dispatching Waypoint -> N: {self.target_n:.2f}, E: {self.target_e:.2f}")
            self.cmd_sent = True
            return Status.RUNNING

        curr_n = self.blackboard.get("curr_n")
        curr_e = self.blackboard.get("curr_e")
        distance = math.sqrt((self.target_n - curr_n)**2 + (self.target_e - curr_e)**2)

        if distance > 2.0:  
            return Status.RUNNING
            
        self.node.get_logger().info("[Flight Status] Target coordinate arrived successfully.")
        
        # Only advance the checkpoint sequence when reaching a true target checkpoint
        if not self.is_detour:
            idx = self.blackboard.get("wp_index")
            self.node.get_logger().info(f"[Milestone Clear] Progressing past Checkpoint Index {idx}.")
            self.blackboard.set("wp_index", idx + 1)

        return Status.SUCCESS

# ==============================================================================
# MAP-AWARE BEHAVIOR TREE GENERATOR
# ==============================================================================
class MapAwareRouter(py_trees.behaviour.Behaviour):
    def __init__(self, name="MapAwareRouter", node=None):
        super().__init__(name)
        self.node = node  
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="wp_index", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="curr_n", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="curr_e", access=py_trees.common.Access.READ)

    def update(self):
        idx = self.blackboard.get("wp_index")
        if idx >= len(TARGET_WAYPOINTS):
            self.node.get_logger().info("[Mission Success] All maze objectives cleared successfully.")
            return Status.SUCCESS

        curr_n = self.blackboard.get("curr_n")
        curr_e = self.blackboard.get("curr_e")
        final_target = TARGET_WAYPOINTS[idx]

        cmd_n, cmd_e, is_detour = calculate_safe_vector(
            curr_n, curr_e, final_target["N"], final_target["E"], OBSTACLES
        )

        if is_detour:
            self.node.get_logger().warn(f"[Safety Warning] Straight path to Index {idx} blocked. Injecting detour leg.")
        else:
            self.node.get_logger().info(f"[Router Status] Straight path verified clear to Checkpoint Index {idx}.")

        parent_composite = self.parent
        branch = py_trees.composites.Sequence(name="Maneuver Leg", memory=False)
        
        action_node = SafeWaypointAction(
            name="Execute_Maneuver",
            target_n=cmd_n,
            target_e=cmd_e,
            altitude=final_target["U"],
            speed=self.node.get_parameter('cruise_speed').value,
            is_detour=is_detour,
            node=self.node
        )
        
        branch.add_child(action_node)
        parent_composite.replace_child(self, branch)
        self.node.reset_router_flag = True
        return Status.SUCCESS  

# ==============================================================================
# MASTER EXECUTIVE CONTAINER
# ==============================================================================
class BTNavMaster(Node):
    def __init__(self):
        super().__init__('bt_maze_navigator')
        
        self.declare_parameter('cruise_speed', 6.0)

        self.state_sub = self.create_subscription(State, 'estimated_state', self.state_cb, 10)
        self.waypoint_client = self.create_client(AddWaypoint, '/path_planner/add_waypoint')
        
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.set("curr_n", 0.0)
        self.blackboard.set("curr_e", 0.0)
        self.blackboard.set("wp_index", 0) 
        
        self.reset_router_flag = False

        self.root = py_trees.composites.Sequence(name="Map-Aware Root", memory=False)
        self.active_router = MapAwareRouter(name="Map Decision Matrix", node=self)
        self.root.add_child(self.active_router)
        
        self.tree_manager = py_trees.trees.BehaviourTree(self.root)
        self.tree_manager.setup(timeout=15.0)

        self.create_timer(0.1, self.loop_callback)
        self.get_logger().info("[System Online] Occupancy Map-Aware Autonomous Brain Initialized.")

    def state_cb(self, msg):
        self.blackboard.set("curr_n", msg.p_n)
        self.blackboard.set("curr_e", msg.p_e)

    def loop_callback(self):
        if self.reset_router_flag and self.root.children[0].status == Status.SUCCESS:
            idx = self.blackboard.get("wp_index")
            if idx >= len(TARGET_WAYPOINTS):
                return
                
            self.root.remove_all_children()
            self.active_router = MapAwareRouter(name="Map Decision Matrix", node=self)
            self.root.add_child(self.active_router)
            self.reset_router_flag = False

        self.tree_manager.tick()

def main(args=None):
    rclpy.init(args=args)
    node = BTNavMaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
