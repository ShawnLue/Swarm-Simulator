import numpy as np
from math import sqrt, atan2, asin, cos, sin, pi


def distance(pos1, pos2):
    """Compute E-distance for 2D"""
    return sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) + 0.001


def reach(p1, p2, bound=0.5):
    """if the robot has reached the destination"""
    return distance(p1, p2) < bound


def compute_V_des(X, goal, V_max):
    """compute goal vector"""
    V_des = []
    for i in xrange(len(X)):
        if reach(X[i], goal[i], 0.1):
            V_des.append([0, 0])
            continue
        dif_x = [goal[i][k] - X[i][k] for k in range(2)]
        norm = distance(dif_x, [0, 0])
        norm_dif_x = [dif_x[k] * V_max[k] / norm for k in range(2)]
        V_des.append(norm_dif_x[:])
    return V_des


def RVO_update(X, V_des, V_current, ws_model, VO_method='VO'):
    """compute best velocity given the desired velocity, current velocity and workspace model"""
    ROB_RAD = ws_model['robot_radius'] + 0.1
    V_opt = list(V_current) # deep copy
    for i in range(len(X)): # Distributed
        vA = [V_current[i][0], V_current[i][1]]
        pA = [X[i][0], X[i][1]]
        RVO_BA_all = []
        # For each other robots
        for j in range(len(X)):
            if i == j:
                continue
            vB = [V_current[j][0], V_current[j][1]]
            pB = [X[j][0], X[j][1]]
            # if distance(pA, pB) > 3.0:
            #     continue
            dist_BA = distance(pA, pB)
            theta_BA = atan2(pB[1] - pA[1], pB[0] - pA[0])
            if 2 * ROB_RAD > dist_BA:
                dist_BA = 2 * ROB_RAD
            theta_BA_ort = asin(2 * ROB_RAD / dist_BA)

            theta_ort_left = theta_BA + theta_BA_ort
            theta_ort_right = theta_BA - theta_BA_ort

            bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
            bound_right = [cos(theta_ort_right), sin(theta_ort_right)]

            transl_vB_vA = [0, 0]
            if VO_method == 'RVO':
                transl_vB_vA = [pA[0] + 0.5 * (vB[0] + vA[0]), pA[1] + 0.5 * (vB[1] + vA[1])]
            # use VO
            elif VO_method == 'VO':
                transl_vB_vA = [pA[0] + vB[0], pA[1] + vB[1]]
            elif VO_method == 'HRVO': # TODO?
                dist_dif = distance([0.5 * (vB[0] - vA[0]), 0.5 * (vB[1] - vA[1])], [0, 0])
                transl_vB_vA = [pA[0] + vB[0] + cos(theta_ort_left) * dist_dif,
                                pA[1] + vB[1] + sin(theta_ort_left) * dist_dif]
            else:
                raise ValueError()

            RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2 * ROB_RAD]
            RVO_BA_all.append(RVO_BA)

        for hole in ws_model['circular_obstacles']:
            # hole = [x, y, rad]
            vB = [0, 0]
            pB = hole[0:2]

            dist_BA = distance(pA, pB)
            theta_BA = atan2(pB[1] - pA[1], pB[0] - pA[0])

            # over-approximation of square of circular
            OVER_APPROX_C2S = 1.5
            rad = hole[2] * OVER_APPROX_C2S
            if (rad + ROB_RAD) > dist_BA:
                dist_BA = rad + ROB_RAD
            theta_BA_ort = asin((rad + ROB_RAD) / dist_BA)

            theta_ort_left = theta_BA + theta_BA_ort
            theta_ort_right = theta_BA - theta_BA_ort

            bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
            bound_right = [cos(theta_ort_right), sin(theta_ort_right)]

            transl_vB_vA = [pA[0] + vB[0], pA[1] + vB[1]]
            RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad + ROB_RAD]
            RVO_BA_all.append(RVO_BA)

        vA_post = intersect(pA, V_des[i], RVO_BA_all)
        V_opt[i] = vA_post[:]
    return V_opt


def intersect(pA, vA, RVO_BA_all):
    # determine the optimal velocity given all VO area and desired velocity
    norm_v = distance(vA, [0, 0])
    suitable_v = []
    unsuitable_v = []
    for theta in np.arange(0, 2 * pi, 0.1):  # TODO: Grid search?
        for rad in np.arange(0.02, norm_v + 0.02, norm_v / 5.0):
            new_v = [rad * cos(theta), rad * sin(theta)]
            suit = True
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left_angle = RVO_BA[1]
                right_angle = RVO_BA[2]
                dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]

                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right_angle[1], right_angle[0])
                theta_left = atan2(left_angle[1], left_angle[0])

                if in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break
            if suit:
                suitable_v.append(new_v)
            else:
                unsuitable_v.append(new_v)
    # for desired velocity
    new_v = vA[:]
    suit = True
    for RVO_BA in RVO_BA_all:
        p_0 = RVO_BA[0]
        left_angle = RVO_BA[1]
        right_angle = RVO_BA[2]
        dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]

        theta_dif = atan2(dif[1], dif[0])
        theta_right = atan2(right_angle[1], right_angle[0])
        theta_left = atan2(left_angle[1], left_angle[0])

        if in_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    if suit:
        suitable_v.append(new_v)
    else:
        unsuitable_v.append(new_v)

    if suitable_v:
        vA_post = min(suitable_v, key=lambda v: distance(v, vA))  # the nearest velocity
        new_v = vA_post[:]
        for RVO_BA in RVO_BA_all:
            p_0 = RVO_BA[0]
            left_angle = RVO_BA[1]
            right_angle = RVO_BA[2]
            dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]

            theta_dif = atan2(dif[1], dif[0])
            theta_right = atan2(right_angle[1], right_angle[0])
            theta_left = atan2(left_angle[1], left_angle[0])
    else:
        # print 'Suitable not found'
        tc_V = dict()   # time collision
        for unsuit_V in unsuitable_v:
            tc_V[tuple(unsuit_V)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left_angle = RVO_BA[1]
                right_angle = RVO_BA[2]
                dist_BA = RVO_BA[3]
                rad = RVO_BA[4]
                dif = [unsuit_V[0] + pA[0] - p_0[0], unsuit_V[1] + pA[1] - p_0[1]]

                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right_angle[1], right_angle[0])
                theta_left = atan2(left_angle[1], left_angle[0])

                if in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif - 0.5 * (theta_left + theta_right))
                    if abs(dist_BA * sin(small_theta)) >= rad:
                        rad = abs(dist_BA * sin(small_theta))
                    big_theta = asin(abs(dist_BA * sin(small_theta)) / rad)
                    dist_tg = abs(dist_BA * cos(small_theta)) - abs(rad * cos(big_theta))
                    dist_tg = (dist_tg > 0) and dist_tg or 0
                    tc_vi = dist_tg / distance(dif, [0, 0])
                    tc.append(tc_vi)
            tc_V[tuple(unsuit_V)] = min(tc) + 0.001
        w_T = 0.2
        vA_post = min(unsuitable_v, key=lambda v: (w_T / tc_V[tuple(v)] + distance(v, vA)))
    return vA_post


def in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= pi:
        return theta_right <= theta_dif <= theta_left
    else:
        if (theta_left < 0) and (theta_right > 0):
            theta_left += 2 * pi
            if theta_dif < 0:
                theta_dif += 2 * pi
            return theta_right <= theta_dif <= theta_left
        if (theta_left > 0) and (theta_right < 0):
            theta_right += 2 * pi
            if theta_dif < 0:
                theta_dif += 2 * pi
            return theta_left <= theta_dif <= theta_right
