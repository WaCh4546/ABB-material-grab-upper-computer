import pyswarms as ps
import numpy as np
import random
import math

def rotate_matrix(yaw, pitch, roll):
# the input euler angle uses degree as unit, which shall be translated into rad before sin and cos calculation.
    yaw = yaw * np.pi / 180.0
    pitch = pitch * np.pi / 180.0
    roll = roll * np.pi / 180.0

    m_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
    m_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
    m_roll = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
    return m_yaw @ m_pitch @ m_roll


class PSO_Solver(object):
    def __init__(self, init_pos_euler, eye_samples, hand_samples):
        self.__eye_samples = np.array(eye_samples)
        self.__hand_samples = np.array(hand_samples)

        # generate initial particles
        particle_cnt = 100
        pos_range = 1000
        euler_range = 30
        init = [init_pos_euler]
        for i in range(particle_cnt - 1):
            init.append(
                (init_pos_euler[0] + random.uniform(-pos_range, pos_range),
                 init_pos_euler[1] + random.uniform(-pos_range, pos_range),
                 init_pos_euler[2] + random.uniform(-pos_range, pos_range),
                 init_pos_euler[3] + random.uniform(-euler_range, euler_range),
                 init_pos_euler[4] + random.uniform(-euler_range, euler_range),
                 init_pos_euler[5] + random.uniform(-euler_range, euler_range))
            )
        init_pos = np.array(init)

        options = {'c1': 1.5, 'c2': 1.5, 'w': 0.5}
        optimizer = ps.single.GlobalBestPSO(n_particles=particle_cnt,
                                            dimensions=6,
                                            options=options,
                                            init_pos=init_pos)
        cost, pos_euler = optimizer.optimize(self.__opt_func, iters=500)

        self.cost = cost
        self.pos_euler = pos_euler

    @staticmethod
    def __calc_hand(pos_euler, eye_samples):
        rm = rotate_matrix(pos_euler[3], pos_euler[4], pos_euler[5])
        t = np.array((pos_euler[0], pos_euler[1], pos_euler[2]))
        return (rm @ eye_samples.T).T + t

    @staticmethod
    def __calc_cost(pos_euler, eye_samples, hand_samples):
        calc_hand = PSO_Solver.__calc_hand(pos_euler, eye_samples)
        diff = (hand_samples - calc_hand) ** 2
        cost = math.sqrt(np.sum(diff) / diff.shape[0])
        return cost

    def __opt_func(self, X):
        n_particles = X.shape[0]  # number of particles
        dist = [PSO_Solver.__calc_cost(X[i], self.__eye_samples, self.__hand_samples) for i in range(n_particles)]
        return np.array(dist)

