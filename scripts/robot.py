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


		----------MDP------------#

		#FIRST EXPERIMENT		
		self.policy_list_nonopt= [[0 for x in range(self.col)] for y in range(self.row)]
		self.policy_list_opt = [[0 for x in range(self.col)] for y in range(self.row)]
		self.iterations = 20
		self.mdp = MDP()
		print "*** Creating policy grid with moving obstacles ***\n"
		self.mdp.make_policy(False, False, False, self.iterations, self.policy_list_nonopt)
		print "*** Creating optimal policy ***\n"
		self.mdp.make_policy(True, False, False, self.max_iterations, self.policy_list_opt)
		self.compare_policies()


		#SECOND EXPERIMENT
		self.policy_list_nonopt= [[0 for x in range(self.col)] for y in range(self.row)]
		self.policy_list_opt = [[0 for x in range(self.col)] for y in range(self.row)]
		self.iterations = 20
		self.mdp = MDP()
		print "*** Creating non-optimal grid and run the robot ***\n"
		self.mdp.make_policy(False, True, False, self.iterations, self.policy_list_nonopt)
		print "\n*** Creating optimal grid and run the robot ***\n"
		self.mdp.make_policy(True, True, False, self.max_iterations, self.policy_list_opt)


		#THIRD EXPERIMENT
		self.policy_list_nonopt= [[0 for x in range(self.col)] for y in range(self.row)]
		self.policy_list_opt = [[0 for x in range(self.col)] for y in range(self.row)]
		self.iterations = 20
		self.mdp = MDP()
		print "*** Creating non-optimal grid and run the robot with increasing uncertainty ***\n"
		self.mdp.make_policy(False, True, True, self.iterations, self.policy_list_nonopt)
		print "*** Creating optimal grid and run the robot with increasing uncertainty ***\n"
		self.mdp.make_policy(True, True, True, self.max_iterations, self.policy_list_opt)


		rospy.sleep(1)
		self.sim_complete_pub.publish(True)
		rospy.sleep(1)
		rospy.signal_shutdown(Robot)


	def compare_policies (self):
		diff = 0
		same = 0
		print "***** Comparing the optimal vs. non-optimal policy grids... ***** "
		for r in range (self.row):
			for c in range (self.col):
				if (self.policy_list_opt[r][c] != self.policy_list_nonopt[r][c]):
					diff = diff + 1
				else:
					same = same + 1

		print "\tNumber of SAME grid policies: ", same
		print "\tNumber of DIFFERENT grid policies: ", diff


if __name__ == '__main__':
   r = Robot()