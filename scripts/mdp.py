#!/usr/bin/env pythn
import rospy
import random
import math
from copy import deepcopy
import numpy as np
from std_msgs.msg import String, Float32, Bool
from read_config import read_config
from cse_190_final.msg import *
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

		

	def make_policy (self, optimal, max_iterations, policy_list):
		self.init_grid()
		self.publish_iteration()
		# going through iter-1 because init takes care of k=0
		for i in range (max_iterations-1):
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
			#publish policy
			if (optimal == False):
				self.move_walls()	
			self.publish_iteration()
			if (optimal == True and stop_iter):
				break
		
		for r in range (self.col):
			for w in range(self.col):
				policy_list[r][w] = self.policy_list[r][w]

		self.current_pos = self.init_src()
		self.print_map()
		while (self.current_pos[0] != self.goal[0] or self.current_pos[1] != self.goal[1]):
			move = self.get_move (self.policy_list[self.current_pos[0]][self.current_pos[1]])
			self.handle_move_request(move)
			self.print_map()
			if (self.current_pos in self.pits):
				#self.print_map()
				print "GAME OVER.YOU FAILED THE MISSION"
				return
			if (self.current_pos[0] == self.goal[0] and self.current_pos[1] == self.goal[1]):
				#self.print_map()
				print "YOU WIN!"
				return

	def init_src (self):
		r = random.randint(0,self.row-1)
		c = random.randint(0,self.col-1)
		while ([r,c] in self.walls):
			r = random.randint(0,self.row-1)
			c = random.randint(0,self.col-1)
		return [r,c]
			


 	def handle_move_request(self, move):
        	print self.config['prob_move_correct']
        	if self.config['uncertain_motion']:
            		roll = random.uniform(0,1)
            		if roll < self.config["prob_move_correct"]:
                		self.make_move(move)
            		else:
                		possible_moves = deepcopy(self.config['move_list'])
                		possible_moves.remove(move)
                		random_move = random.choice(possible_moves)
                		self.make_move(random_move)
        	elif not self.config['uncertain_motion']:
            		self.make_move(move)


    	def make_move(self, move):
		pos = [0,0]
        	pos[0] = (self.current_pos[0] + move[0]) 
        	pos[1] = (self.current_pos[1] + move[1]) 
		if (pos not in self.walls):
			if((pos[0] >= self.row or pos[0] <= -1) or (pos[1] >= self.col or pos[1] <= -1)):
				return
			self.current_pos[0] = pos[0]
			self.current_pos[1] = pos[1]
        	
		elif (pos in self.walls):
			self.config['prob_move_correct'] = self.config['prob_move_correct'] - 0.05
			if (self.config['prob_move_correct'] < 0.7):
        			self.config['prob_move_correct'] = 0.7
			return
		
			

	def get_move (self, direction):
		if ( direction == "N" ):
			return [-1,0]
		if ( direction == "S" ):
			return [1,0]
		if ( direction == "W" ):
			return [0,-1]
		if ( direction == "E" ):
			return [0,1]


	def print_map (self):
		for row in range (self.row):
			for col in range (self.col):
				if([row, col] in self.walls):
					print "# |",
				elif(row == self.goal[0] and col == self.goal[1]):
					print "G |",
				elif([row, col] in self.pits):
					print "X |",
				elif(row == self.current_pos[0] and col == self.current_pos[1]):
					print "R |",
				else:
					print "  |",
			print ""
			
			for c in range (self.col):
				print "--",
			print ""	
		print " "


	def move_walls(self):
		new_walls = []
		for i in (self.walls):
			collision = False
			while ( collision == False ):
				x = random.randint(-2, 2)
				y = random.randint(-2, 2)

				while((i[0]+x) <0 or (i[0]+x) >= self.row):
					x = random.randint(-2, 2)
				while((i[1] +y) <0 or (i[1]+y) >= self.col):
					y = random.randint(-2, 2)

				if(x == 2 or x == 1 or x == -1 or x == -2):	
					y = 0

				x += i[0]
				y += i[1]

				if([x,y] not in new_walls and (x != self.goal[0] and y != self.goal[1]) and [x,y] not in self.pits):
					self.policy_list[x][y] = "WALL"
					self.grid[x][y] = 0
					self.policy_list[i[0]][i[1]] = "N"
					self.grid[i[0]][i[1]] = 0
					new_walls.append([x,y])
					collision = True
		self.walls = []
		for j in (new_walls):
			self.walls.append(deepcopy(j))


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

		for w in (self.walls):
			self.policy_list[w[0]][w[1]] = "WALL"

		for i in range (self.row):
			for j in range (self.col):
				array.append(self.policy_list[i][j])	
		
		msg.data = array
		rospy.sleep(1)
		self.mdp_pub.publish(msg)
