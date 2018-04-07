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
traj = np.stack((traj_x, traj_y, traj_z)).T

traj = utils.interpolate_path(traj, T, dt)  # interpolate path
traj_d = utils.calc_derivatives(traj, dt)  # calculate velocity

tk = 1
robot.q = np.array([np.pi/3, -2*np.pi/3, np.pi/3])
# robot.init_arm(np.array([np.pi/2, 0, 0]))
# q, pos_end_effector = robot.follow_trajectory(traj, traj_d, T, dt)

while tk < N:
    x = np.array([traj[tk, 0], traj[tk, 1], traj[tk, 2]])
    xd = np.array([traj_d[tk, 0], traj_d[tk, 1], traj_d[tk, 2]])
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

plt.show()
