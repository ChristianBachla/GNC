from scipy import linalg as la
import matplotlib.pyplot as pl
import numpy as np
import helper_functions as hp

from pylab import figure, show, rand
from matplotlib.patches import Ellipse, PathPatch
from matplotlib.transforms import Affine2D
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import mpl_toolkits.mplot3d.art3d as art3d

from leader import leader as lead
from follower import follower as fol
import quadrotor as quad
import quadlog
import animation as ani

# Quadrotor
m = 0.65 # Kg
l = 0.23 # m
Jxx = 7.5e-3 # Kg/m^2
Jyy = Jxx
Jzz = 1.3e-2
Jxy = 0
Jxz = 0
Jyz = 0
J = np.array([[Jxx, Jxy, Jxz], \
              [Jxy, Jyy, Jyz], \
              [Jxz, Jyz, Jzz]])
CDl = 9e-3
CDr = 9e-4
kt = 3.13e-5  # Ns^2
km = 7.5e-7   # Ns^2
kw = 1/0.18   # rad/s

# Initial conditions
att_0 = np.array([0.0, 0.0, 0.0])
pqr_0 = np.array([0.0, 0.0, 0.0])
xyz1_0 = np.array([2.0, 2.0, 0.0])
xyz2_0 = np.array([1.0, 1.0, 0.0])
v_ned_0 = np.array([0.0, 0.0, 0.0])
w_0 = np.array([0.0, 0.0, 0.0, 0.0])

# Setting quads
q1 = lead(1, m, l, J, CDl, CDr, kt, km, kw, att_0, pqr_0, xyz1_0, v_ned_0, w_0)
q2 = fol(2, m, l, J, CDl, CDr, kt, km, kw, att_0, pqr_0, xyz2_0, v_ned_0, w_0)

# Simulation parameters
tf = 2000
dt = 1e-2
time = np.linspace(0, tf, tf/dt)
it = 0
frames = 100

# Data log
q1_log = quadlog.quadlog(time)
q2_log = quadlog.quadlog(time)

# Plots
quadcolor = ['k', 'g']
pl.close("all")
pl.ion()
fig = pl.figure(0)
axis3d = fig.add_subplot(111, projection='3d')

pl.figure(0)

# Desired position and heading
xyz1_d = np.array([10, 10, -10])
xyz2_d = np.array([0, 0, -10])

q1.yaw_d = -np.pi/4
q2.yaw_d = -np.pi/4

mid_vec_x = []
mid_vec_y = []
q1_vec_x = []
q1_vec_y = []
q2_vec_x = []
q2_vec_y = []
error_vec = []
d = []
for t in time:

    #
    # Simulation
    q1.run(xyz1_d, dt, q2)
    q2.run(xyz2_d, dt, q1)

    #parameters for end graphs.
    if q1.e != None:
        error_vec.append(q1.e)
    xs = q2.xyz[0], q1.xyz[0]
    ys = q2.xyz[1], q1.xyz[1]
    zs = -q2.xyz[2], -q1.xyz[2]

    midpoint = hp.midpoint(q1.xyz, q2.xyz)
    if it % 1000 ==0:
        mid_vec_x.append(midpoint[0])
        mid_vec_y.append(midpoint[1])
        q1_vec_x.append(q1.xyz[0])
        q1_vec_y.append(q1.xyz[1])
        q2_vec_x.append(q2.xyz[0])
        q2_vec_y.append(q2.xyz[1])


    line = art3d.Line3D(ys, xs, zs)


    # Animation


    if it%frames == 0:

        '''
        axis3d.cla()
        ani.draw3d(axis3d, q1.xyz, q1.Rot_bn(), quadcolor[0])
        ani.draw3d(axis3d, q2.xyz, q2.Rot_bn(), quadcolor[1])
        e = Ellipse((0, 0), 10, 20, 0, fill=0)
        axis3d.add_patch(e)
        axis3d.add_line(line)
        ani.draw3d(axis3d, midpoint, (q1.Rot_bn() - q2.Rot_bn()), quadcolor[0])

        art3d.pathpatch_2d_to_3d(e, z=10, zdir='z')
        axis3d.set_xlim(-10, 10)
        axis3d.set_ylim(-10, 10)
        axis3d.set_zlim(0, 15)
        axis3d.set_xlabel('South [m]')
        axis3d.set_ylabel('East [m]')
        axis3d.set_zlabel('Up [m]')
        axis3d.set_title("Time %.3f s" %t)
        pl.pause(0.001)
        pl.draw()
        '''



    # Log

    q1_log.xyz_h[it, :] = q1.xyz
    q1_log.att_h[it, :] = q1.att
    q1_log.w_h[it, :] = q1.w
    q1_log.v_ned_h[it, :] = q1.v_ned
    q1_log.xi_g_h[it] = q1.xi_g
    q1_log.xi_CD_h[it] = q1.xi_CD

    q2_log.xyz_h[it, :] = q2.xyz
    q2_log.att_h[it, :] = q2.att
    q2_log.w_h[it, :] = q2.w
    q2_log.v_ned_h[it, :] = q2.v_ned
    q2_log.xi_g_h[it] = q2.xi_g
    q2_log.xi_CD_h[it] = q2.xi_CD


    it+=1

    # Stop if crash
    if (q1.crashed == 1 or q2.crashed ==1):
        break

u_m = np.diff(mid_vec_x)
v_m = np.diff(mid_vec_y)
u_q1 = np.diff(q1_vec_x)
v_q1 = np.diff(q1_vec_y)
u_q2 = np.diff(q2_vec_x)
v_q2 = np.diff(q2_vec_y)

pos_x_m = mid_vec_x[:-1] + u_m/2
pos_y_m = mid_vec_y[:-1] + v_m/2
norm_m = np.sqrt(u_m**2 + v_m**2)

pos_x_q1 = q1_vec_x[:-1] + u_q1/2
pos_y_q1 = q1_vec_y[:-1] + v_q1/2
norm_q1 = np.sqrt(u_q1**2 + v_q1**2)

pos_x_q2 = q2_vec_x[:-1] + u_q2/2
pos_y_q2 = q2_vec_y[:-1] + v_q2/2
norm_q2 = np.sqrt(u_q2**2 + v_q2**2)

pl.figure(1)
pl.plot(time, q1_log.w_h[:, 0], label="w_1_q1")
pl.plot(time, q1_log.w_h[:, 1], label="w_2_q1")
pl.plot(time, q1_log.w_h[:, 2], label="w_3_q1")
pl.plot(time, q1_log.w_h[:, 3], label="w_4_q1")
pl.plot(time, q2_log.w_h[:, 0], label="w_1_q2")
pl.plot(time, q2_log.w_h[:, 1], label="w_2_q2")
pl.plot(time, q2_log.w_h[:, 2], label="w_3_q2")
pl.plot(time, q2_log.w_h[:, 3], label="w_4_q2")
pl.xlabel("Time [s]")
pl.ylabel("Motor angular velocity [rad/s]")
pl.grid()
pl.legend()

pl.figure(2)
pl.plot(time, q1_log.att_h[:, 0], label="roll_q1")
pl.plot(time, q1_log.att_h[:, 1], label="pitch_q1")
pl.plot(time, q1_log.att_h[:, 2], label="yaw_q1")
pl.plot(time, q2_log.att_h[:, 0], label="roll_q2")
pl.plot(time, q2_log.att_h[:, 1], label="pitch_q2")
pl.plot(time, q2_log.att_h[:, 2], label="yaw_q2")
pl.xlabel("Time [s]")
pl.ylabel("Attitude angle [rad]")
pl.grid()
pl.legend()

pl.figure(3)
pl.plot(time, -q1_log.xyz_h[:, 2], label="UP_q1")
pl.plot(time, q1_log.xyz_h[:, 0], label="X_q1")
pl.plot(time, q1_log.xyz_h[:, 1], label="Y_q1")
pl.plot(time, -q2_log.xyz_h[:, 2], label="UP_q2")
pl.plot(time, q2_log.xyz_h[:, 0], label="X_q2")
pl.plot(time, q2_log.xyz_h[:, 1], label="Y_q2")
pl.xlabel("Time [s]")
pl.ylabel("Position [m]")
pl.grid()
pl.legend()

pl.figure(4)
pl.plot(time, -q1_log.v_ned_h[:, 2], label="-V_z_q1")
pl.plot(time, q1_log.v_ned_h[:, 0], label="V_x_q1")
pl.plot(time, q1_log.v_ned_h[:, 1], label="V_y_q1")
pl.plot(time, -q2_log.v_ned_h[:, 2], label="-V_z_q2")
pl.plot(time, q2_log.v_ned_h[:, 0], label="V_x_q2")
pl.plot(time, q2_log.v_ned_h[:, 1], label="V_y_q2")
pl.xlabel("Time [s]")
pl.ylabel("Velocity [m/s]")
pl.grid()
pl.legend()

pl.figure(5)
pl.plot(time, q1_log.xi_g_h, label="${\\xi}_g$_q1")
pl.plot(time, q1_log.xi_CD_h, label="${\\xi}_{CD}$_q1")
pl.plot(time, q2_log.xi_g_h, label="${\\xi}_g$_q2")
pl.plot(time, q2_log.xi_CD_h, label="${\\xi}_{CD}$_q2")
pl.xlabel("Time [s]")
pl.ylabel("Estimators value")
pl.grid()
pl.legend()

#Error plots
fig, (ax1, ax2) = pl.subplots(1, 2)
fig.suptitle('Error distance of Q1 and the trajectory of Q1, Q2 and their centre of gravety')
ax1.plot(time, error_vec, label='Error' )
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Error value")
pl.grid()

#Plots of translation of rotorcrafts
ax2.quiver(pos_x_m, pos_y_m, u_m/norm_m, v_m/norm_m, angles="xy",color = 'g', zorder=5, pivot="mid", scale=6, scale_units='inches', label='Midpoint')
ax2.quiver(pos_x_q1, pos_y_q1, u_q1/norm_q1, v_q1/norm_q1, angles="xy",color = 'r', zorder=5, pivot="mid", scale=6, scale_units='inches', label='Q1 Trajectory')
ax2.quiver(pos_x_q2, pos_y_q2, u_q2/norm_q2, v_q2/norm_q2, angles="xy",color = 'b', zorder=5, pivot="mid", scale=6, scale_units='inches', label='Q2 Trajectory')
ax2.set_xlim(-12, 12)
ax2.set_ylim(-12, 12)
e = Ellipse((0, 0), 20, 10, 0, fill=0)
ax2.add_artist(e)
ax2.set_xlabel("X values")
ax2.set_ylabel("Y values")
ax2.grid()
ax2.legend()


pl.grid()
#pl.show()


pl.pause(0)
