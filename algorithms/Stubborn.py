# -*- coding:utf-8 -*-
# __author__ = 'shawnlue'
from BaseAlgorithm import BaseAgent
from math import pi


class Stubborn(BaseAgent):
    def __init__(self):
        super(Stubborn, self).__init__()

    def policy(self, state_list):
        command_list = [(1.0, pi / 3) for _ in range(len(state_list))]
        return command_list
