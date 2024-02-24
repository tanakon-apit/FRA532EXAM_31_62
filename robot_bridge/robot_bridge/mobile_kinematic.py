#!/usr/bin/python3

import numpy as np

class Diff_Drive_Kinematic():
    def __init__(self, r : float, b : float):
        self.Tt2w = np.array([1.0 / r, -0.5 * b / r,
                              1.0 / r, 0.5 * b / r]).reshape(2, 2)
        self.Tw2t = np.array([0.5 * r, 0.5 * r,
                              0.0, 0.0, 
                              -r / b, r / b]).reshape(3, 2)
        self.pos_g = np.zeros(3)
        self.qd = np.zeros(2)

    def get_wheelspeed(self, vel_r : list):
        self.qd = self.Tt2w @ np.array(vel_r)
        return self.qd

    def get_pose(self, dt : float):
        Rr2g = np.array([np.cos(self.pos_g[2]), -np.sin(self.pos_g[2]), 0.0,
                         np.sin(self.pos_g[2]), np.cos(self.pos_g[2]), 0.0,
                         0.0, 0.0, 1.0]).reshape(3, 3)
        vel_g = Rr2g @ self.Tw2t @ self.qd
        self.pos_g += vel_g * dt
        return self.pos_g

    def get_twist(self):
        return np.linalg.inv(self.Tt2w) @ self.qd