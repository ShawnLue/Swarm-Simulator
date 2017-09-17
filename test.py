from simulator.Navigation.Environment import Environment
from simulator.default_settings import *

from algorithms.Stubborn import Stubborn
from algorithms.RVO_Policy import RVO_update

WIDTH = 500
HEIGHT = 500

# Initialize experiment
env = Environment(WIDTH, HEIGHT, centered=True, gui=True, use_mouse=USE_MOUSE)

ob_num, robot_num = 8, 15
# s_set, ob_set = env.reset(ob_num, robot_num)
s_set, ob_set = env.get_map_1()

# tmp1 = env.get_map_1()

# Initialize agent
# agent = Stubborn()

count = 0
while env.running:
    # command = agent.policy(s_set)
    command = RVO_update(s_set, ob_set, 'HRVO')
    s_set, ob_set = env.step(command)
    # print s_set[1]['pos'], s_set[1]['goal'], s_set[1]['id']
    # print s_set[-2]['pos'], s_set[-2]['goal'], s_set[-2]['id']
    # exit()

    # print s_set[0], ob_set[0]

    # if count == 200:
    #     exit()
    count += 1