# -*- coding:utf-8 -*-
# __author__ = 'shawnlue'


class BaseAgent(object):

    def __init__(self):
        pass

    def policy(self, state_list):
        return [(0.0, 0.0) for _ in range(len(state_list))]

    def _reinforce(self):
        pass
