#!/usr/bin/env python
import rospy
import random as r
import math
import numpy as np
from read_config import read_config
from std_msgs.msg import String, Float32, Bool
from cse_190_assi_3.msg import *
import heapq
#from copy import deepcopy, copy


class Astar():
    def __init__(self):
	self.config = read_config()
	self.move_list = self.config['move_list']
	self.start = self.config['start']
	self.goal = self.config['goal']
	self.walls = self.config['walls']
	self.pits = self.config['pits']
	self.map_size = self.config['map_size']
	self.row = self.map_size[0]	
	self.col = self.map_size[1]	

	self.heuristics_grid = [[0 for x in range(self.col)] for y in range(self.row)]
	self.parent_array = [[0 for x in range(self.col)] for y in range(self.row)]
	self.f_array =  [[0 for x in range(self.col)] for y in range(self.row)]
	self.uniform_grid = [[0 for x in range(self.col)] for y in range(self.row)]
	self.heuristics()
	
	self.open_list = []
	heapq.heapify(self.open_list)
	self.closed_list = set()
	#self.path_array = []


    def heuristics (self):
	for i in range (self.row):
		for j in range (self.col):
			value1 = math.fabs(i - self.goal[0])
			value2 = math.fabs(j - self.goal[1])
			self.heuristics_grid[i][j] = value1 + value2


    def astar_func(self, result_array):
	self.f_array[self.start[0]][self.start[1]] = 0		
	heapq.heappush(self.open_list, (self.f_array[self.start[0]][self.start[1]], self.start[0], self.start[1]))
	self.uniform_grid[self.start[0]][self.start[1]] = 0
	while len(self.open_list):
		current_f, current_x, current_y = heapq.heappop(self.open_list)
		self.closed_list.add ( (current_x, current_y) )
		if ( current_x == self.goal[0] and current_y == self.goal[1] ):
			self.create_path(result_array)
			return
		neighbors_array = self.getNeighbors(current_x, current_y)
		for i in range (len(neighbors_array)):
			x, y = neighbors_array[i]
		        new_path = self.heuristics_grid[x][y] + self.uniform_grid[x][y]
			if (x, y) in self.closed_list:	
				continue	
			if( self.f_array[x][y] > new_path or (self.f_array[x][y], x, y) not in self.open_list):
				self.f_array[x][y] = new_path	
				self.parent_array[x][y] = [current_x, current_y]
				if (self.f_array[x][y], x, y) not in self.open_list:
					heapq.heappush( self.open_list, (self.f_array[x][y], x, y) )


    # go through the parent_array and append the paths to self.path_array
    def create_path(self, path_array):
	path_array.append(self.goal)
	row = self.goal[0]
	col = self.goal[1]
	while ( self.parent_array[row][col] != self.start ):
		path_array.append(self.parent_array[row][col])
		array = self.parent_array[row][col]
		row = array[0]
		col = array[1]

	path_array.append( self.start )
	path_array.reverse()


    def getNeighbors(self, x, y):
	neighbors = []		
	if ( [0,-1] in self.move_list and  y - 1 >= 0 and [x,y-1] not in self.walls and [x,y-1] not in self.pits ):
		neighbors.append( (x, y-1) )	
		self.uniform_grid[x][y-1] = self.uniform_grid[x][y]+1

	if ( [0,1] in self.move_list and y + 1 < self.col and [x,y+1] not in self.walls and [x,y+1] not in self.pits ):
		neighbors.append( (x, y+1) )	
		self.uniform_grid[x][y+1] = self.uniform_grid[x][y]+1

	if ( [-1,0] in self.move_list and x - 1 >= 0 and [x-1,y] not in self.walls and [x-1,y] not in self.pits):
		neighbors.append( (x-1, y) )	
		self.uniform_grid[x-1][y] = self.uniform_grid[x][y]+1

	if ( [1,0] in self.move_list and x + 1 < self.row and [x+1,y] not in self.walls and [x+1,y] not in self.pits ) :
		neighbors.append( (x+1, y) )	
		self.uniform_grid[x+1][y] = self.uniform_grid[x][y]+1
		
	return neighbors
