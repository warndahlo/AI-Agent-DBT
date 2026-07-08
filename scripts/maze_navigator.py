#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import math
from std_msgs.msg import Float32MultiArray
from roscopter_msgs.msg import Waypoint, State
from roscopter_msgs.srv import AddWaypoint

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')
        
        self.wall_sub = self.create_subscription(
            Float32MultiArray,
            'sensors/walls_sensor',
            self.wall_callback,
            10)
            
        self.state_sub = self.create_subscription(
            State,
            'estimated_state',
            self.state_callback,
            10)
        
        self.waypoint_client = self.create_client(AddWaypoint, '/path_planner/add_waypoint')
            
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /path_planner/add_waypoint service...')
        
        self.get_logger().info('Dynamic Velocity Shaping Brain Engaged!')

        self.d_north, self.d_east, self.d_south, self.d_west = 0.0, 0.0, 0.0, 0.0
        self.curr_n, self.curr_e = 0.0, 0.0
        self.target_n = 0.0
        self.target_e = 0.0
        self.current_axis = 'east' 
        self.state = 'TAKING_OFF'
        self.last_decision_time = time.time()

        # Telemetry
        self.leg_start_time = None
        self.leg_counter = 0

    def send_waypoint(self, n, e):
        req = AddWaypoint.Request()
        wp_msg = Waypoint()
        wp_msg.type = 1  
        wp_msg.w = [n, e, -6.0]  
        wp_msg.psi = 0.0         
        wp_msg.clear_wp_list = False  
        
        # Calculate straight-line distance to this new target
        distance = math.sqrt((n - self.curr_n)**2 + (e - self.curr_e)**2)
        
        # DYNAMIC VELOCITY SHAPING:
        # If the waypoint is far, lower commanded speed so the PID doesn't choke out the thrust vector.
        # If it is close, give it full speed to clear the turn.
        if distance > 15.0:
            wp_msg.speed = 25.0  # Sweet-spot velocity for high-efficiency, flat straightaway cruising
        else:
            wp_msg.speed = 20.0 # Standard speed for short hops
        
        req.wp = wp_msg
        req.publish_now = True
        
        self.leg_counter += 1
        self.leg_start_time = time.time()
        
        self.get_logger().info(f"==> LEG {self.leg_counter} SENT -> Target North: {n:.2f}, East: {e:.2f} | Assigned Speed: {wp_msg.speed:.1f}")
        self.waypoint_client.call_async(req)

    def wall_callback(self, msg):
        self.d_north = msg.data[0]
        self.d_east  = msg.data[1]
        self.d_south = msg.data[2]
        self.d_west  = msg.data[3]

    def state_callback(self, msg):
        self.curr_n = msg.p_n
        self.curr_e = msg.p_e

        now = time.time()
        if now - self.last_decision_time < 0.2:
            return
        self.last_decision_time = now

        if self.state == 'TAKING_OFF':
            if self.d_north > 0.0:
                self.get_logger().info("Takeoff confirmed. Calculating first hallway track...")
                self.make_navigation_decision()
            return

        if self.state == 'WAITING':
            dist_thresh = 0.5
            if (abs(self.curr_n - self.target_n) < dist_thresh) and (abs(self.curr_e - self.target_e) < dist_thresh):
                if self.leg_start_time is not None:
                    elapsed = time.time() - self.leg_start_time
                    self.get_logger().info(f"LEG {self.leg_counter} FINISHED! Time taken: {elapsed:.2f} seconds.")
                
                self.get_logger().info("Corner milestone hit! Swapping tracking axis...")
                self.make_navigation_decision()

    def make_navigation_decision(self):
        if self.d_north == 0.0:
            return

        BUFFER = 4.0  

        if self.current_axis != 'north':
            if (self.d_north > self.d_south) or (self.d_south > 9999):
                self.target_n = self.curr_n + self.d_north - BUFFER
            else:
                self.target_n = self.curr_n - self.d_south + BUFFER
            
            self.target_e = self.curr_e
            self.current_axis = 'north'

        elif self.current_axis != 'east':
            if (self.d_east > self.d_west) or (self.d_west > 9999):
                self.target_e = self.curr_e + self.d_east - BUFFER
            else:
                self.target_e = self.curr_e - self.d_west + BUFFER
            
            self.target_n = self.curr_n
            self.current_axis = 'east'

        self.state = 'WAITING'
        self.send_waypoint(self.target_n, self.target_e)


def main():
    rclpy.init()
    node = MazeSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
