#!/usr/bin/env python3

"""
biped_sim.launch.py

launch the BipedalSim Node
to simulate the gym environement
for other ROS2 nodes to control with
"""

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_descrpition():
    return LaunchDescription([
        Node(
            package='box2d_rl',
            executable='bipedal_simulator.py'
        ),
    ])