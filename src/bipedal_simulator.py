import rclpy
from rclpy import Node
import gym
import numpy as np

"""
bipedal_simulator.py

node that handles the simulation
"""

class BipedalSim(Node):
    def __init__(self):
        self.action_sub = rclpy.create_subscription(
            #Action_Message,
            '/action',
            self.action_callback,
            10)
        self.env = gym.make("BipedalWalker-v2", render_mode="human")