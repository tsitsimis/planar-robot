import numpy as np


class PlanarArm:
    def __init__(self, links, pos0=np.array([0, 0, 0])):
        self.links = links
        self.pos0 = pos0
        self.n_links = links.shape[0]

        self.pos = np.zeros((3, self.n_links))
        self.q = np.zeros(self.n_links)
        self.q_d = np.zeros(self.n_links)

        self.Jac = None

    def forward_kinematics(self, q):
        pos = np.zeros((3, self.n_links))
        pos[:, 0] = self.pos0 + np.array([[self.links[0] * np.cos(q[0]), self.links[0] * np.sin(q[0]), 0]])
        for i in range(1, self.n_links):
            delta_pos = np.array([self.links[i] * np.cos(np.sum(q[0:i+1])),
                                  self.links[i] * np.sin(np.sum(q[0:i+1])), 0])
            pos[:, i] = pos[:, i - 1] + delta_pos

        self.pos = pos
        return pos

    def jacobian(self, q):
        jac = np.zeros((3, self.n_links))

        w1 = -self.links * np.array([np.sin(np.sum(q[0:i+1])) for i in range(self.n_links)])
        w2 = self.links * np.array([np.cos(np.sum(q[0:i+1])) for i in range(self.n_links)])

        jac[0, :] = [np.sum(w1[i::]) for i in range(self.n_links)]
        jac[1, :] = [np.sum(w2[i::]) for i in range(self.n_links)]
        jac[2, :] = np.zeros(self.n_links)
        self.Jac = jac
        return jac

    def inverse_diff_kinematics(self, x, xd, dt, Kp=10):
        self.Jac = self.jacobian(self.q)
        jac = self.Jac
        jac_pseudo = np.dot(jac.T, np.linalg.inv(np.dot(jac, jac.T) + 0.001 * np.eye(3)))

        self.q_d = np.dot(jac_pseudo, xd + Kp*(x - self.pos[:, -1]))

        self.q = self.q + self.q_d * dt
        self.pos = self.forward_kinematics(self.q)
        return self.q, self.pos

    def follow_trajectory(self, traj, traj_d, T, dt, Kp=10):
        t = np.arange(0, T, dt)
        N = t.shape[0]
        q = np.zeros((N, self.n_links))
        pos_end_effector = np.zeros((N, 3))
        for i in range(N):
            x = np.array([traj[0, i], traj[1, i], traj[2, i]])
            xd = np.array([traj_d[0, i], traj_d[1, i], traj_d[2, i]])
            q[i, :], pos = self.inverse_diff_kinematics(x, xd, dt, Kp)
            pos_end_effector[i, :] = pos[:, -1]
        return q, pos_end_effector

    def init_arm(self, q, q_d=None):
        self.q = q
        self.forward_kinematics(q)

        if q_d is not None:
            self.q_d = np.dot(self.jacobian(q), q_d)

    def plot(self, plt):
        plt.scatter(self.pos0[0], self.pos0[1], c='k', zorder=10, s=100)
        plt.plot([self.pos0[0], self.pos[0, 0]], [self.pos0[1], self.pos[1, 0]], c='orange')
        for i in range(0, self.n_links):
            plt.scatter(self.pos[0, i], self.pos[1, i], c='blue', zorder=10)
            if i >= 1:
                plt.plot([self.pos[0, i - 1], self.pos[0, i]], [self.pos[1, i - 1], self.pos[1, i]], c='orange')