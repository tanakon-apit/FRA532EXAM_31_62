#!/usr/bin/python3

import numpy as np

class Diff_Drive_Cov_Estimator():
    def __init__(self, kr : float, kl : float):
        self.k = np.array([kr, 0.0,
                           0.0, kl]).reshape(2, 2)
        self.p_cov = np.zeros([3, 3])
        
    def update_cov(self, theta : float, dsr : float, dsl : float):
        ds = (dsr + dsl) / 2.0
        dtheta = (dsr - dsl) / 2.0
        s_cov = self.k
        s_cov[0][0] *= dsr
        s_cov[1][1] *= dsl

        a0 = np.cos(theta + (dtheta / 2.0))
        a1 = np.sin(theta + (dtheta / 2.0))

        grad_pf = np.eye(3)
        grad_pf[0][2] = -ds * a1
        grad_pf[1][2] = ds * a0

        grad_drlf = np.array([0.5 * a0, 0.5 * a0,
                              0.5 * a1, 0.5 * a0,
                              0.5, -0.5]).reshape(3, 2)
        
        self.p_cov = (grad_pf @ (self.p_cov @ grad_pf.T)) + (grad_drlf @ (s_cov @ grad_drlf.T))
        return self.p_cov
