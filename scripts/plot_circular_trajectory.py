import numpy as np
import matplotlib.pyplot as plt

def circular_traj(init, radius, n_points):
    xi, yi = init[0], init[1]
    center = (init[0]+radius, init[1])

    traj = []
    x_vals = []
    y_vals = []

    angles = np.linspace(0, np.pi, n_points)

    for angle in angles:
        traj.append((center[0] + radius*np.cos(angle), center[1] + radius*np.sin(angle)))
        x_vals.append(center[0] + radius*np.cos(angle))
        y_vals.append(center[1] + radius*np.sin(angle))

    # Plot the trajectory
    plt.figure(figsize=(10, 10))
    plt.plot(x_vals, y_vals, 'b-', label='Semi-Circular Path')
    
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Semi-Circular Foot Trajectory')
    plt.scatter([xi], [yi], color='red', label='Start Position')
    plt.legend()
    plt.grid()
    
    plt.show()
    
    return traj

points = circular_traj(init=(0, -0.45), radius=0.1, n_points=100)
print(points)