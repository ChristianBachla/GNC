import numpy as np
from scipy import linalg as la
from quadrotor import quadrotor
import helper_functions as hp
from math import cos, sin

E = np.array([[0, 1], [-1, 0]])

#Ellipse parameter for COM to follow
O_e = [10, 5]
O = np.array([0, 0])

#gains
ke = 0.2 * 5
ks = 0.08 * 5

#Leader knows own xyz,
class leader(quadrotor):
    def __init__(self, tag, m, l, J, CDl, CDr, kt, km, kw, att, pqr, xyz, v_ned, w):
        quadrotor.__init__(self, tag, m, l, J, CDl, CDr, kt, km, kw, att, pqr, xyz, v_ned, w)
        self.step1 = 1
        self.gvf = np.array([])
        self.scenario = 0
        self.e = 0
        self.ke = ke
        self.ks = ks


    def run(self, xyz_d, dt, q2):

        if self.step1 == 1:
            self.set_xyz_ned_lya(xyz_d)

            #Setting scenario depending on start position of Q1
            p = np.array([self.xyz[0], self.xyz[1]])
            if (p[0] < 0 and p[1] < 0):
                self.scenario = 0
            if (p[0] > 0 and p[1] > 0):
                self.scenario = 1
            if (p[0] < 0 and p[1] > 0):
                self.scenario = 2
            if (p[0] > 0 and p[1] < 0):
                self.scenario = 3

            if q2.ready == 1:
                self.step1 = 2

        elif self.step1 == 2:
            self.follow_ellipsis(q2)

        self.step(dt)

    def follow_ellipsis(self, q2, alt_d = -10):

        p = np.array([self.xyz[0], self.xyz[1]])

        #Chentre of ellipse to track
        O = hp.get_distance_rel(self.xyz, q2.xyz)
        O = O / 2.0

        #error distance to own ellipsis
        e_distance = hp.get_error_distance(p, O, O_e)
        self.e = e_distance #variable used for end plot

        #normal vector
        n_vector = hp.get_normal_vector(p, O, O_e)

        #Tangent vector
        tau_p = E.dot(n_vector) * self.ks

        #Guidance vector field
        gvf = self.get_GVF(tau_p, e_distance, n_vector)
        self.gvf = gvf
        u = gvf

        self.set_v_2D_alt_lya(u, alt_d)


    def get_GVF(self, tau, e, n):
        gvf = tau - self.ke * e * n
        return gvf
