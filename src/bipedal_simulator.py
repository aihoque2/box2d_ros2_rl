import rclpy
from rclpy import Node
import gymnasium as gym
import numpy as np

from std_msgs.msg import Float64MultiArray, Float64
"""
bipedal_simulator.py

node that handles the simulation of bipedalWalker

this is just proof of concept of reinforcement learning
using controllers implemented with ROS2.
"""

class BipedalSim(Node):
    def __init__(self):
        super().__init__('bipedal_sim')

        """
        action_sub takes action msg and does env.step(msg)
        reward_pub publishes the reward recieved after step()
        timer_callback for consistent rendering of the environment
        """
    
        self.action_sub = rclpy.create_subscription(
            # Action_Message,
            '/action',
            self.action_callback,
            10)

        self.state_pub = rclpy.create_publisher(Float64MultiArray, "/state", 10)
        self.reward_pub = rclpy.create_publisher(Float64, "/reward", 10)
        
        self.env = gym.make("BipedalWalker-v3", render_mode="human")
        self.state_size = self.env.observation_space.shape[0]
        self.action_size = self.env.action_space
        self.state, info = self.env.reset()

        timer_period = 0.017 # publish rate of 60 messages/second?
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def action_callback(self, msg: Float64MultiArray):
        """
        send the force message and update
        """

        action = np.array(msg)
        new_state, reward, terminated, truncated, info = self.env.step(action)
        
        # TODO: do something with the reward and other vars
        self.state = new_state
        state_msg = Float64MultiArray(self.state)
        reward = Float64(reward) # cast reward to ROS2 msg
        self.reward_pub.publish(reward)
        self.state_pub.publish(state_msg)

        if terminated or truncated:
            state, info = self.env.reset()
            self.state = state
            self.state_pub.publish(state_msg)

    
    def timer_callback(self):
        """
        TODO: WTF IS A TIMER?

        publishes the state and renders env
        """
        state_msg = Float64MultiArray(self.state)
        self.state_pub.publish(state_msg)
        self.env.render()

