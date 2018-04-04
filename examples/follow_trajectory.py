import matplotlib.pyplot as plt
import numpy as np

from planarobot import utils
from planarobot.planar_arm import PlanarArm

# robot
links = np.ones(3)
# links = np.random.random(15)
robot = PlanarArm(links, pos0=np.array([2, 2, 0]))

# animation
T = 1.0
dt = 1e-3
t = np.arange(0, T, dt)
N = t.shape[0]

# desired trajectory
traj_x = 2 * (1 - t)
traj_y = np.sqrt(4 - traj_x**2)
traj_z = np.zeros(N)
traj = np.stack((traj_x, traj_y, traj_z))

traj_x_ = utils.interpolate_path(traj_x, T, dt)
traj_y_ = utils.interpolate_path(traj_y, T, dt)
traj_z_ = utils.interpolate_path(traj_z, T, dt)

traj_xd = utils.calc_derivatives(traj_x_, dt)
traj_yd = utils.calc_derivatives(traj_y_, dt)
traj_zd = utils.calc_derivatives(traj_z_, dt)
traj_d = np.stack((traj_xd, traj_yd, traj_zd))

tt = 0
tk = 1
robot.q = np.array([np.pi/3, -2*np.pi/3, np.pi/3])
# robot.init_arm(np.array([np.pi/2, 0, 0]))
# q, pos_end_effector = robot.follow_trajectory(traj, traj_d, T, dt)

while tk < N:
    x = np.array([traj_x[tk], traj_y[tk], traj_z[tk]])
    xd = np.array([traj_xd[tk], traj_yd[tk], traj_zd[tk]])
    _, pos = robot.inverse_diff_kinematics(x, xd, dt)

    # plot
    if np.mod(tk, 10) == 0 or tk == 1:
        plt.cla()
        plt.plot(traj_x, traj_y, c='k', linestyle='--')

        plt.scatter(robot.pos0[0], robot.pos0[1], c='k', zorder=10, s=100)
        plt.plot([robot.pos0[0], pos[0, 0]], [robot.pos0[1], pos[1, 0]], c='orange')
        for i in range(0, robot.n_links):
            plt.scatter(pos[0, i], pos[1, i], c='blue', zorder=10)
            if i >= 1:
                plt.plot([pos[0, i - 1], pos[0, i]], [pos[1, i - 1], pos[1, i]], c='orange')

        plt.draw()
        plt.pause(0.05)
        plt.axis('equal')
        plt.axis('square')

    tk += 1
    tt += dt

plt.show()
