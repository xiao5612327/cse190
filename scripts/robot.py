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
		self.mdp = MDP()
		self.mdp.make_policy()

		rospy.sleep(1)
		self.sim_complete_pub.publish(True)
		rospy.sleep(1)
		rospy.signal_shutdown(Robot)


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
