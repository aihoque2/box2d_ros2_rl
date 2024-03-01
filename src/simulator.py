import rclpy
import gym
import numpy as np

"""
simulator.py

node that handles the simulation
"""

class Box2DSim(Node):
    def __init__(self):
        self.action_sub = rclpy.create_subscription(
            #Action_Message,
            '/action',
            self.action_callback,
            10)
        self.env = gym.make("BipedalWalker-v2", render_mode="human")