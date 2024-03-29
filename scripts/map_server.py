#!/usr/bin/env python

import rospy
import random as r
import math as m
import numpy as np
from copy import deepcopy
from cse_190_final.srv import  moveService
from read_config import read_config


class MapServer():
    def __init__(self):
        """Read config file and setup ROS things"""
        self.config = read_config()
        self.config["prob_move_correct"] = .80
        rospy.init_node("map_server")
    
        self.move_service = rospy.Service(
                "moveService",
                moveService,
                self.handle_move_request
        )
        self.pos = self.initialize_position()

    def initialize_position(self):
        pos = self.config["starting_pos"]
        return pos

    def handle_move_request(self, request):
        move = list(request.move)
        if self.config['uncertain_motion']:
            roll = r.uniform(0,1)
            if roll < self.config["prob_move_correct"]:
                self.make_move(move)
            else:
                possible_moves = deepcopy(self.config['move_list'])
                possible_moves.remove(move)
                random_move = r.choice(possible_moves)
                self.make_move(random_move)
        elif not self.config['uncertain_motion']:
            self.make_move(move)
        return self.pos

    def make_move(self, move):
        num_rows = self.config['map_size'][0]
        num_cols = self.config['map_size'][1]
        self.pos[0] = (self.pos[0] + move[0]) % num_rows
        self.pos[1] = (self.pos[1] + move[1]) % num_cols


if __name__ == '__main__':
    ms = MapServer()
