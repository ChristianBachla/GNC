import numpy as np
from scipy import linalg as la
from quadrotor import quadrotor
import helper_functions as hp
from math import cos, sin

kc = 0.08


class follower(quadrotor):
    def __init__(self, tag, m, l, J, CDl, CDr, kt, km, kw, att, pqr, xyz, v_ned, w):
        quadrotor.__init__(self, tag, m, l, J, CDl, CDr, kt, km, kw, att, pqr, xyz, v_ned, w)
        self.step2 = 0
        self.dt_mat = np.array([])
        self.ready = 0

    def run(self, xyz_d, dt, q1):

        if self.step2 == 2:
            displacement = hp.get_displacement(self.xyz, q1.xyz)
            self.follow_leader(displacement, q1, q1.scenario)

        if self.step2 == 0:
            self.set_xyz_ned_lya(xyz_d)

            if (self.xyz[2] + 10) < 0.01:
                self.step2 = 1

        if self.step2 == 1:
            displacement = hp.get_displacement(self.xyz, q1.xyz)
            self.consensus(displacement, q1.scenario)

            if (displacement < 2.01 and displacement > 1.99):
                self.step2 = 2
                self.ready = 1

        self.step(dt)

    #Consensus algorithm
    def consensus(self, displacement, scenario):

        if scenario == 0:
            u1 = kc * np.sum(0.5*(2 - displacement))
            u2 = kc * np.sum(0.5*(2 - displacement))

        if scenario == 1:
            u1 = kc * np.sum(0.5*(displacement - 2))
            u2 = kc * np.sum(0.5*(displacement - 2))

        if scenario == 2:
            u1 = kc * np.sum(0.5*(2 - displacement))
            u2 = kc * np.sum(0.5*(displacement - 2))

        if scenario == 3:
            u1 = kc * np.sum(0.5*(displacement - 2))
            u2 = kc * np.sum(0.5*(2 - displacement))

        u = np.array([u1, u2])
        self.set_v_2D_alt_lya(u, -10)

    #Function to follow leader (Same as above plus GVF)
    def follow_leader(self, displacement, q1, scenario):
        if any(q1.gvf) != 0:

            if scenario == 0:
                u1 = kc * np.sum(0.5*(2 - displacement)) + q1.gvf[0]
                u2 = kc * np.sum(0.5*(2 - displacement)) + q1.gvf[1]

            if scenario == 1:
                u1 = kc * np.sum(0.5*(displacement - 2)) + q1.gvf[0]
                u2 = kc * np.sum(0.5*(displacement - 2)) + q1.gvf[1]

            if scenario == 2:
                u1 = kc * np.sum(0.5*(2 - displacement)) + q1.gvf[0]
                u2 = kc * np.sum(0.5*(displacement - 2)) + q1.gvf[1]

            if scenario == 3:
                u1 = kc * np.sum(0.5*(displacement - 2)) + q1.gvf[0]
                u2 = kc * np.sum(0.5*(2 - displacement)) + q1.gvf[1]

            u  = np.array([u1, u2])
            self.set_v_2D_alt_lya(u, -10)

    def ready(self, q1):
        return q1.ready
