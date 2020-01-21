import numpy as np
from scipy import linalg as la
import sys, termios, tty, os, time

def midpoint(p1, p2):
    mx = (p1[0] + p2[0]) / 2
    my = (p1[1] + p2[1]) / 2
    mz = (p1[2] + p2[2]) / 2
    midpoint = (mx, my, mz)
    return midpoint

def get_error_distance(p, O, L_W):
    e = pow(p[0] - O[0],2)/pow(L_W[0],2) + pow(p[1] - O[1],2)/pow(L_W[1],2) - 1
    return e

def get_normal_vector(p, O, L_W):
    n_path_x = (2*(p[0] - O[0]))/pow(L_W[0],2)
    n_path_y = (2*(p[1] - O[1]))/pow(L_W[1],2)
    n_p = np.array([n_path_x, n_path_y])
    return n_p

def get_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def get_distance_rel(p1, p2):
    return np.array([p1[0] - p2[0], p1[1] - p2[1]])

def get_displacement(p1, p2):
    p1 = np.array(p1[0], p1[1])
    p2 = np.array(p2[0], p2[1])

    return la.norm(p1 - p2)
