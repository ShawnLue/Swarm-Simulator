from RVO import compute_V_des, RVO_update
from vis import visualize_traj_dynamic
import numpy as np
from math import sqrt

VISIUALIZATION = False
method = 'VO'

# ----------Environment Initialization------------------------ #

# define workspace model
ws_model = dict()

# robot radius
ws_model['robot_radius'] = 0.2

# circular obstacles, format [x, y, rad]
# if no obstacles
# ws_model['circular_obstacles'] = []
# with obstacles
ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3],
                                  [1.5, 2.5, 0.3],
                                  [3.3, 2.5, 0.3],
                                  [5.1, 2.5, 0.3]]
# rectangular boundary, format [x, y, width/2, height/2]
ws_model['boundary'] = []

# -----------Robot Initialization----------------------- #
# position of [x, y]
X = [[-0.5 + 1.0 * i, 0.0] for i in range(7)] + [[-0.5 + 1.0 * i, 5.0] for i in range(7)]

# velocity of [vx, vy]
V = [[0, 0] for i in range(len(X))]

# maximal velocity norm
V_max = [1.0 for i in range(len(X))]

# goal of [x, y]
goal = [[5.5 - 1.0 * i, 5.0] for i in range(7)] + [[5.5 - 1.0 * i, 0.0] for i in range(7)]

# -----------Simulation Setup----------------------- #
# total simulation time (s)
total_time = 20

# simulation step
step = 0.01

# -----------Simulation----------------------- #
t = 0

total_dis = 0.0

while t * step < total_time:
    print t
    # compute desired vel to goal
    V_des = compute_V_des(X, goal, V_max)
    if np.unique(V_des)[0] == 0:
        print t * step
        break
    # compute the optimal vel to avoid collision
    V = RVO_update(X, V_des, V, ws_model, VO_method=method)
    # update position
    for i in range(len(X)):
        X[i][0] += V[i][0] * step
        X[i][1] += V[i][1] * step
        total_dis += sqrt((V[i][0] * step) ** 2 + (V[i][1] * step) ** 2)
    # visualization
    if VISIUALIZATION and t % 10 == 0:
        visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.pdf'%str(t/10))
    t += 1

print total_dis
