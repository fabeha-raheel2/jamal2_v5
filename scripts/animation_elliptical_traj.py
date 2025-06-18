import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

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

    def animate_trajectory(self):
        # Trajectory parameters
        x_pre = 0.00
        y_pre = -0.45
        z_pre = 0.105
        L = -0.05
        H = 0.08
        W = 0.00

        t_range = np.linspace(0, np.pi, 100)

        # Generate forward and backward trajectories
        x_f, y_f, z_f = self.elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, self.trapezoidal_profile, t_range)
        x_b, y_b, z_b = self.translation(x_f[-1], x_f[0], y_pre, y_pre, z_pre, z_pre, t_range)

        # Combine them into one full loop
        x_all = np.concatenate((x_f, x_b))
        y_all = np.concatenate((y_f, y_b))
        z_all = np.concatenate((z_f, z_b))

        # Set up the figure and axis
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_title('Quadruped Leg Trajectory')

        ax.set_xlim([min(x_all)-0.01, max(x_all)+0.01])
        ax.set_ylim([min(z_all)-0.01, max(z_all)+0.01])
        ax.set_zlim([min(y_all)-0.01, max(y_all)+0.01])

        # Trajectory line and moving point
        traj_line, = ax.plot([], [], [], 'gray', lw=1, label="Trajectory")
        point, = ax.plot([], [], [], 'ro', label='Foot')

        def init():
            traj_line.set_data([], [])
            traj_line.set_3d_properties([])
            point.set_data([], [])
            point.set_3d_properties([])
            return traj_line, point

        def update(i):
            traj_line.set_data(x_all[:i+1], z_all[:i+1])
            traj_line.set_3d_properties(y_all[:i+1])

            point.set_data(x_all[i], z_all[i])
            point.set_3d_properties(y_all[i])
            return traj_line, point

        ani = FuncAnimation(fig, update, frames=len(x_all), init_func=init,
                            interval=30, blit=True)
        
        ax.legend()
        plt.show()

if __name__ == "__main__":
    traj = Trajectory()
    traj.animate_trajectory()
