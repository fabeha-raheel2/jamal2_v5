import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, t_range):
    x = x_pre + L * np.cos(np.pi - t_range)
    y = y_pre + H * np.sin(np.pi - t_range)
    z = z_pre + W * np.cos(np.pi - t_range)
    return x, y, z

def translation_along_x_axis(x_pre, y_pre, z_pre, L, W, t_range):
    x = np.linspace(x_pre + 2 * L, x_pre, len(t_range))
    y = np.full_like(x, y_pre)
    z = np.linspace(z_pre + 2 * W, z_pre, len(t_range))
    return x, y, z

def plot_trajectory():
    x_pre, y_pre, z_pre = 0.0, -0.30, 0.0905  # Rest position
    L, H, W = -0.05, 0.08, 0.00  # Motion parameters
    t_range = np.linspace(0, np.pi, 100)
    
    x_traj_f, y_traj_f, z_traj_f = elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, t_range)
    x_traj_b, y_traj_b, z_traj_b = translation_along_x_axis(x_pre, y_pre, z_pre, L, W, t_range)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Quadruped Leg Trajectory')
    ax.set_xlim([x_pre + 2*L, x_pre])
    ax.set_ylim([-0.4, 0])
    ax.set_zlim([z_pre - 0.1, z_pre + 0.1])
    
    ax.plot(x_traj_f, y_traj_f, z_traj_f, 'r-', label='Forward Step')
    ax.plot(x_traj_b, y_traj_b, z_traj_b, 'b-', label='Backward Step')
    ax.legend()
    
    plt.show()
    
if __name__ == "__main__":
    plot_trajectory()
