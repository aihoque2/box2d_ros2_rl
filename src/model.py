#!/usr/local/bin/python3

import gym, os
import numpy as np

from itertools import count
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Categorical

EPS = 0.003

def fanin_init(size, fanin=None):
	"""
	set the weights in some random numbers 
	uniformly distributed
	"""
	fanin = fanin or size[0]
	v = 1. / np.sqrt(fanin)
	return torch.Tensor(size).uniform_(-v, v)


class Actor(nn.Module):
	
	def __init__(self, state_size, action_size, action_lim, hidden1=500, hidden2=400, hidden3=200):
		
		super(Actor, self).__init__()
		
		self.action_lim: np.float32 = action_lim #enforce a type
		self.state_size = state_size
		self.action_size = action_size

        #initialize the model's layers
		self.fc1 = nn.Linear(state_size, hidden1) #input layer that takes in our read state
		self.fc1.weight.data = fanin_init(self.fc1.weight.data.size()) #initialize random variables from a uniform distribution
		
		self.fc2 = nn.Linear(hidden1, hidden2)
		self.fc2.weight.data = fanin_init(self.fc2.weight.data.size())
		
		self.fc3 = nn.Linear(hidden2, hidden3)
		self.fc3.weight.data = fanin_init(self.fc3.weight.data.size())
		
		self.fc4 = nn.Linear(hidden3, action_size)
		self.fc4.weight.data.uniform_(-EPS, EPS)

		self.relu = nn.ReLU()

	def forward(self, state):
		"""
    	This is our policy function, 𝛑(s) 
    	Takes in state, returns an action
    	to use.
    	"""
		x = self.fc1(state)
		x = self.relu(x)
		x = F.relu(self.fc2(x))
		x = F.relu(self.fc3(x))
		action = F.tanh(self.fc4(x))
		action = action * self.action_lim
		
		return action  