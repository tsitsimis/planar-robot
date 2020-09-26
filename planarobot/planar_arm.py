import numpy as np


class PlanarArm:
    def __init__(self, links, base_pos=np.array([0, 0])):
        self.links = links
        self.base_pos = base_pos
        self.n_links = links.shape[0]

        self.q = np.zeros(self.n_links)
        self.q_d = np.zeros(self.n_links)
        self.pos = np.zeros((self.n_links, 2))

    def forward_kinematics(self, q):
        """
        Calculates links positions given joint angles

        Parameters
        ----------
        q : numpy.ndarray
            (n_links,) array of angles in radians
        """

        pos = np.zeros((self.n_links, 2))
        pos[0, :] = self.base_pos + np.array([[self.links[0] * np.cos(q[0]), self.links[0] * np.sin(q[0])]])
        for i in range(1, self.n_links):
            delta_pos = np.array([self.links[i] * np.cos(np.sum(q[0:i+1])),
                                  self.links[i] * np.sin(np.sum(q[0:i+1]))])
            pos[i, :] = pos[i - 1, :] + delta_pos

        return pos

    def jacobian(self, q):
        jac = np.zeros((2, self.n_links))

        w1 = -self.links * np.array([np.sin(np.sum(q[0:i+1])) for i in range(self.n_links)])
        w2 = self.links * np.array([np.cos(np.sum(q[0:i+1])) for i in range(self.n_links)])

        jac[0, :] = [np.sum(w1[i::]) for i in range(self.n_links)]
        jac[1, :] = [np.sum(w2[i::]) for i in range(self.n_links)]
        
        return jac

    def inverse_diff_kinematics(self, x, xd, dt, Kp=10):
        """
        Calculates joint velocities for given end-effector positions and velocities
        """

        jac = self.jacobian(self.q)
        jac_pseudo = np.dot(jac.T, np.linalg.inv(np.dot(jac, jac.T) + 0.001 * np.eye(2)))

        self.q_d = np.dot(jac_pseudo, xd + Kp*(x - self.pos[-1, :]))
        
        self.q = self.q + self.q_d * dt
        self.pos = self.forward_kinematics(self.q)
        return self.q, self.pos

    def trajectory_planning(self, traj, traj_d, dt, Kp=10):
        """
        Applies inverse differential kinematics for given 
        trajectory positions and velocities

        Parameters
        ----------
        traj : numpy.ndarray
            (N, 2) matrix of positions

        traj_d : numpy.ndarray
            (N, 2) matrix of velocities
        """

        N = traj.shape[0]
        robot_pos = np.zeros((N, self.n_links, 2))
        for i in range(N):
            x = traj[i, :]
            xd = traj_d[i, :]
            _, pos = self.inverse_diff_kinematics(x, xd, dt, Kp)
            robot_pos[i, :] = pos
        
        return robot_pos

    def plot(self, pos, ax, arm_width=1, joint_size=5):
        """
        Plot arms and links given their locations

        Parameters
        ----------
        pos : numpy.ndarray
            (N, 2) matrix of coordinates

        ax
            Matplotlib axis object
        """

        ax.scatter(self.base_pos[0], self.base_pos[1], c='k', zorder=10, s=100)
        ax.plot([self.base_pos[0], pos[0, 0]], [self.base_pos[1], pos[0, 1]], c='orange', linewidth=arm_width)
        
        for i in range(self.n_links):
            ax.scatter(pos[i, 0], pos[i, 1], c='blue', zorder=10, s=joint_size)
            if i >= 1:
                ax.plot([pos[i - 1, 0], pos[i, 0]], [pos[i - 1, 1], pos[i, 1]], c='orange', linewidth=arm_width)
