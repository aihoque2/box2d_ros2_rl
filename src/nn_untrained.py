#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import gymnasium as gym
import numpy as np

from std_msgs.msg import Float64MultiArray, Float64
import torch
from model import Actor

"""
nn_untrained.py
# how well can pytorch run? simulate?
"""

device = 'cuda' if torch.cuda.is_available() else 'cpu'

class UntrainedNN(Node):
    def __init__(self):
        super().__init__("random_actions")
        self.action_pub = self.create_publisher(Float64MultiArray, "/action", 10)
        timer_period = 0.017
        self.model = Actor(24, 4, 1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        arr = np.random.uniform(-1.0, 1.0, size=(4,))
        msg = Float64MultiArray(data=list(arr))
        self.action_pub.publish(msg)


if __name__ == "__main__":
    rclpy.init()
    action_node = UntrainedNN()
    i = 0
    while i < 300:
        rclpy.spin_once(action_node)
        i+=1
    action_node.destroy_node()
    rclpy.shutdown()