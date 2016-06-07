#!/usr/bin/env python

import rospy
import random
import math
import numpy as np
from std_msgs.msg import String, Float32, Bool
from read_config import read_config
from cse_190_assi_3.msg import *
from astar import Astar
from mdp import MDP

class Robot ():
	def __init__(self):
		rospy.init_node ("robot")
		self.config = read_config()
		self.map_size = self.config['map_size']
		self.max_iterations = self.config['max_iterations']
		self.row = self.map_size[0]	
		self.col = self.map_size[1]	

		self.astar_pub = rospy.Publisher(
			"/results/path_list",
			AStarPath,
			queue_size = 10
		)

		self.sim_complete_pub = rospy.Publisher(
			"/map_node/sim_complete",
			Bool,
			queue_size = 10
		)

		# publish A* path
		self.path_array = []
		#self.publish_astar()

		# publish MDP
		#print "running mdp"
		
		self.policy_list_nonopt= [[0 for x in range(self.col)] for y in range(self.row)]
		self.policy_list_opt = [[0 for x in range(self.col)] for y in range(self.row)]

		self.mdp = MDP()
		self.mdp.make_policy(False, 20, self.policy_list_nonopt)
#		self.mdp.make_policy(True, self.max_iterations, self.policy_list_opt)

		self.compare_policies()

		rospy.sleep(1)
		self.sim_complete_pub.publish(True)
		rospy.sleep(1)
		rospy.signal_shutdown(Robot)

	def compare_policies (self):
		diff = 0
		same = 0
		for r in range (self.row):
			for c in range (self.col):
				if (self.policy_list_opt[r][c] != self.policy_list_nonopt[r][c]):
					diff = diff + 1
				else:
					same = same + 1

		print "same: ", same
		print "diff: ", diff

	def publish_astar(self):
		obj = Astar()
		obj.astar_func(self.path_array)
		for i in range (len(self.path_array)):
			rospy.sleep(1)
			msg = AStarPath()
			msg.data = self.path_array[i]
			self.astar_pub.publish(msg)
			#self.astar_pub.publish(self.path_array[i])


if __name__ == '__main__':
   r = Robot()
