import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Trajectory:
    def __init__(self):

        self.a = 1.0
        self.T = 3.0
        self.compute_velocity()

    def compute_velocity(self):
        self.t_a = (self.a * self.T) / (2 + self.a)
        self.v = self.a * self.t_a

    def elliptical_trajectory(self, x_pre, y_pre, z_pre, L, H, W, s_func, t_range):

        x = x_pre + L + L * np.cos(np.pi - s_func(t_range))
        y = y_pre + H * np.sin(np.pi - s_func(t_range))
        z = z_pre + W + W * np.cos(np.pi - s_func(t_range))

        return x, y, z

    def linear_displacement(self, t):
        return t

    def trapezoidal_profile(self, t):
        t_a_mask1 = t <= self.t_a
        t_a_mask2 = (t > self.t_a) & (t <= self.T - self.t_a)

        s = np.where(
            t_a_mask1,
            0.5 * self.a * t**2,
            np.where(
                t_a_mask2,
                self.v * t - (self.v**2) / (2 * self.a),
                (2 * self.a * self.v * self.T - 2 * self.v**2 - self.a**2 * (t - self.T)**2) / (2 * self.a)
            )
        )

        s_max = s[-1]
        normalized_s = (s / s_max) * np.pi
        return normalized_s

    def translation(self, x_start, x_end, y_start, y_end, z_start, z_end, t_range):
        x = np.linspace(x_start, x_end, len(t_range))
        y = np.linspace(y_start, y_end, len(t_range))
        z = np.linspace(z_start, z_end, len(t_range))

        return x, y, z

    def plot_trajectory(self):

        x_pre = 0.00     # x-coordinate of end-effector at rest position.
        y_pre = -0.45  # y-coordinate of end-effector at rest position.
        z_pre = 0.105  # z-coordinate of end-effector at rest position.
        L = -0.05   # Decrease to move the end effector forward. Increase to move it backward.
        H = 0.08    # Decrease to move the end effector downward. Increase to move it upward.
        W = 0.00    # Decrease to move the end effector inward. Increase to move it outward.
        t_range = np.linspace(0, np.pi, 100)
        
        x_traj_f, y_traj_f, z_traj_f = self.elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, self.trapezoidal_profile, t_range)
        x_traj_b, y_traj_b, z_traj_b = self.translation(L*2, x_pre, y_pre, y_pre, z_pre, z_pre, t_range) # -0.1, 0.0
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_title('Quadruped Leg Trajectory')
        
        # ax.plot(x_traj_f, y_traj_f, z_traj_f, 'r-', label='Forward Step')
        # ax.plot(x_traj_b, y_traj_b, z_traj_b, 'b-', label='Backward Step')
        ax.plot(x_traj_f, z_traj_f, y_traj_f, 'r-', label='Forward Step')
        ax.plot(x_traj_b, z_traj_b, y_traj_b, 'b-', label='Backward Step')
        ax.legend()
        
        plt.show()
    
if __name__ == "__main__":
    traj = Trajectory()
    traj.plot_trajectory()
