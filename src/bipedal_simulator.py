import rclpy
from rclpy import Node
import gym
import numpy as np

from std_msgs import Float64MultiArray, Float64Array
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
        
        self.state_pub = rclpy.create_publisher(Float64MultiArray, "state", 10)
        self.timer_period = 0.5 # seconds

        self.env = gym.make("BipedalWalker-v3", render_mode="human")
        self.state_size = self.env.observation_space.shape[0]
        self.action_size = self.env.action_space
        self.state, info = self.env.reset()
    
    def action_callback(self, msg: Float64Array):
        """
        send the force message and update
        """

        action = np.array(msg)
        new_state, reward, terminated, truncated, info = self.env.step(action)
        
        # TODO: do something with the reward and other vars
        self.state = new_state
        
        if terminated or truncated:
            state, info = self.env.reset()
            self.state = state
    
    def timer_callback(self):
        pass