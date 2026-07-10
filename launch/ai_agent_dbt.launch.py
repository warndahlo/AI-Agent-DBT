import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    walls_sensor_node = Node(
        package='onboarding_project',
        executable='walls_sensor.py',
        name='walls_sensor',
        output='screen',
    )
    
    walls_publisher_node = Node(
        package='onboarding_project',
        executable='rviz_hallway_publisher',
        name='rviz_hallway_publisher',
        output='screen',
    )

    # ADDED: This kicks off the actual AI behavior tree routing loop
    ai_bt_agent_node = Node(
        package='onboarding_project',
        executable='BT_Ollama_maze_navigator.py',
        name='bt_ollama_master_navigator',
        output='screen',
    )

    return LaunchDescription([
        walls_sensor_node,
        walls_publisher_node,
        ai_bt_agent_node, # Included in the launch sequence
    ])
