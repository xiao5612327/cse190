#!/usr/bin/env pythn
import rospy
import random
import math
import numpy as np
from std_msgs.msg import String, Float32, Bool
from read_config import read_config
from cse_190_assi_3.msg import *
from astar import Astar

class MDP():
	def __init__ (self):
		self.config = read_config()
		self.move_list = self.config['move_list']

		self.max_iterations = self.config['max_iterations']
		self.threshold_difference = self.config['threshold_difference']
		self.step_reward = self.config['reward_for_each_step']
		self.wall_reward = self.config['reward_for_hitting_wall']
		self.goal_reward = self.config['reward_for_reaching_goal']
		self.pit_reward = self.config['reward_for_falling_in_pit']

		
		self.discount_factor = self.config['discount_factor']
		self.prob_move_forward = self.config['prob_move_forward']
		self.prob_move_backward = self.config['prob_move_backward']
		self.prob_move_left = self.config['prob_move_left']
		self.prob_move_right = self.config['prob_move_right']

		self.start = self.config['start']
		self.goal = self.config['goal']
		self.walls = self.config['walls']
		self.pits = self.config['pits']
		self.map_size = self.config['map_size']
		self.row = self.map_size[0]	
		self.col = self.map_size[1]	

		self.mdp_pub = rospy.Publisher(
			"/results/policy_list",
			PolicyList,
			queue_size = 10
		)
		
		self.grid = [[0 for x in range(self.col)] for y in range(self.row)]
		self.policy_list = [[0 for x in range(self.col)] for y in range(self.row)]


	def make_policy (self):
		self.init_grid()
		self.publish_iteration()
		# going through iter-1 because init takes care of k=0
		iteration = 1
		for i in range (self.max_iterations-1):
			#print "iteration ", iteration, ":"
			new_grid = [[0 for x in range(self.col)] for y in range(self.row)]
			for r in range (self.row):
				for c in range (self.col):			
					# do not update the goal or the src too?
					if ([r,c] in self.walls or [r,c] in self.pits or (self.goal[0] == r and self.goal[1] == c)):
						continue
					max_cost = float("-inf")
					best_move = [0,0]
					for a in (self.move_list):
						cost = self.action_cost([r,c],a)
						max_cost = max(max_cost, cost)
						if (max_cost == cost):
							best_move[0] = a[0]
							best_move[1] = a[1]
					new_grid[r][c] = max_cost
				  	self.update_policy(r, c, best_move)	
			stop_iter = self.copy_grid(new_grid)
			#print "about to publish"
			#publish policy
			self.publish_iteration()
			iteration += 1
			
			if (stop_iter):
				return
			
		

	def init_grid (self):
		for i in range (self.row):
			for j in range (self.col):
				self.grid[i][j] = 0
				self.policy_list[i][j] = "N"

		self.grid[self.goal[0]][self.goal[1]] = self.goal_reward
		self.policy_list[self.goal[0]][self.goal[1]] = "GOAL"

		# set pits to have pit rewards intially
		for p in (self.pits):
			self.grid[p[0]][p[1]] = self.pit_reward
			self.policy_list[p[0]][p[1]] = "PIT"

		for w in (self.walls):
			self.policy_list[w[0]][w[1]] = "WALL"


	def action_cost(self, s, a):
		left_prob = 0
		right_prob = 0 
		up_prob = 0
		down_prob = 0

		if ( a[0] == 0 and a[1] == 1 ): #right action	
			left_prob = self.prob_move_backward
			right_prob = self.prob_move_forward
			up_prob = self.prob_move_left
			down_prob = self.prob_move_right
			#print "right"

		elif ( a[0] == 0 and a[1] == -1 ): #left action	
			left_prob = self.prob_move_forward
			right_prob = self.prob_move_backward
			up_prob = self.prob_move_right
			down_prob = self.prob_move_left
			#print "left"

		elif ( a[0] == -1 and a[1] == 0 ): #up action	
			left_prob = self.prob_move_left
			right_prob = self.prob_move_right
			up_prob = self.prob_move_forward
			down_prob = self.prob_move_backward
			#print "up"

		elif ( a[0] == 1 and a[1] == 0 ): #down action	
			left_prob = self.prob_move_right
			right_prob = self.prob_move_left
			up_prob = self.prob_move_backward
			down_prob = self.prob_move_forward
			#print "down"

		reward_to_cell = self.step_reward
		reward_from_cell = 0
		if ( [s[0],s[1]-1] in self.walls or (s[1]-1 < 0) ):
			reward_to_cell = self.wall_reward
			reward_from_cell = self.grid[s[0]][s[1]]
		else:
			reward_from_cell = self.grid[s[0]][s[1]-1]

		left = left_prob* (reward_to_cell + self.discount_factor*reward_from_cell)

		reward_to_cell = self.step_reward
		reward_from_cell = 0
		if ( [s[0],s[1]+1] in self.walls or s[1]+1 >= self.col):
			reward_to_cell = self.wall_reward
			reward_from_cell = self.grid[s[0]][s[1]]
		else:
			reward_from_cell = self.grid[s[0]][s[1]+1]

		right = right_prob * (reward_to_cell + self.discount_factor*reward_from_cell)



		reward_to_cell = self.step_reward
		reward_from_cell = 0
		if ( [s[0]+1,s[1]] in self.walls or s[0]+1 >= self.row):
			reward_to_cell = self.wall_reward
			reward_from_cell = self.grid[s[0]][s[1]]
		else:
			reward_from_cell = self.grid[s[0]+1][s[1]]

		down = down_prob * (reward_to_cell + self.discount_factor*reward_from_cell)


		reward_to_cell = self.step_reward
		reward_from_cell = 0
		if ( [s[0]-1,s[1]] in self.walls or s[0]-1 < 0):
			reward_to_cell = self.wall_reward
			reward_from_cell = self.grid[s[0]][s[1]]
		else:
			reward_from_cell = self.grid[s[0]-1][s[1]]
		up = up_prob * (reward_to_cell + self.discount_factor*reward_from_cell)

		return (left+right+down+up)


	def copy_grid(self, new_grid):
		diff = 0
		for r in range (self.row):
			for c in range (self.col):
				if( [r,c] in self.walls or [r,c] in self.pits or (r == self.goal[0] and c == self.goal[1]) ):
					continue
				diff += abs(self.grid[r][c] - new_grid[r][c])
				self.grid[r][c] = new_grid[r][c]

		# should we stop the value iteration?	
		if (diff <= self.threshold_difference):	
			return True
		else:
			return False


	def update_policy (self, r, c, best_move):
		if (best_move[0] == 0 and best_move[1] == 1): #right
			self.policy_list[r][c] = "E"
		elif (best_move[0] == 0 and best_move[1] == -1): #left
			self.policy_list[r][c] = "W"
		elif (best_move[0] == 1 and best_move[1] == 0): #down
			self.policy_list[r][c] = "S"
		elif (best_move[0] == -1 and best_move[1] == 0): #up
			self.policy_list[r][c] = "N"

	def publish_iteration(self):
		msg = PolicyList()
		array = []
		for i in range (self.row):
			for j in range (self.col):
				array.append(self.policy_list[i][j])	
		
		msg.data = array
		rospy.sleep(1)
		self.mdp_pub.publish(msg)
