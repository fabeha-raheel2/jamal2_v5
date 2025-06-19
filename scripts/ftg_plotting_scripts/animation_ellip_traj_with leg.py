import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# === LINK LENGTHS (in meters) ===
L1 = 0.230
L2 = 0.250

# === TRAJECTORY CLASS ===
class Trajectory:
    def __init__(self, a=1.0, T=3.0):
        self.a = a
        self.T = T
        self.compute_velocity()

    def compute_velocity(self):
        self.t_a = (self.a * self.T) / (2 + self.a)
        self.v = self.a * self.t_a

    def trapezoidal_profile(self, t_range):
        t_a_mask1 = t_range <= self.t_a
        t_a_mask2 = (t_range > self.t_a) & (t_range <= self.T - self.t_a)

        s = np.where(
            t_a_mask1,
            0.5 * self.a * t_range**2,
            np.where(
                t_a_mask2,
                self.v * t_range - (self.v**2) / (2 * self.a),
                (2 * self.a * self.v * self.T - 2 * self.v**2 - self.a**2 * (t_range - self.T)**2) / (2 * self.a)
            )
        )

        s_max = s[-1]
        normalized_s = (s / s_max) * np.pi
        return normalized_s

    def elliptical_trajectory(self, t_range):
        # Ellipse parameters (you can tweak these)
        xc, yc = 0.100, -0.325
        a = 0.250 / 2
        b = 0.175 / 2
        s_func = self.trapezoidal_profile
        s_vals = s_func(t_range)

        x = xc - a * np.cos(np.pi - s_vals)
        y = yc + b * np.sin(np.pi - s_vals)
        return x, y

# === MAIN SCRIPT ===
# Time vector
dt = 0.01
T_total = 3.0
t = np.arange(0, T_total, dt)

# Trajectory generation
traj = Trajectory(T=T_total)
x_elliptical, y_elliptical = traj.elliptical_trajectory(t)
x_elliptical = x_elliptical[::-1]
y_elliptical = y_elliptical[::-1]

# Reverse straight-line return
x_straight = np.linspace(x_elliptical[-1], x_elliptical[0], len(x_elliptical))
y_straight = np.linspace(y_elliptical[-1], y_elliptical[0], len(y_elliptical))

# Combine full cycle
x_total = np.concatenate((x_elliptical, x_straight))
y_total = np.concatenate((y_elliptical, y_straight))

# === INVERSE KINEMATICS ===
theta1 = np.zeros_like(x_total)
theta2 = np.zeros_like(x_total)

for i in range(len(x_total)):
    xi, yi = x_total[i], y_total[i]
    D = (xi**2 + yi**2 - L1**2 - L2**2) / (2 * L1 * L2)
    D = np.clip(D, -1.0, 1.0)  # for numerical stability
    theta2[i] = np.arccos(D)
    theta1[i] = np.arctan2(yi, xi) - np.arctan2(L2 * np.sin(theta2[i]), L1 + L2 * np.cos(theta2[i]))

# === ANIMATION ===
fig, ax = plt.subplots()
ax.set_xlim(-0.5, 0.6)
ax.set_ylim(-0.6, 0.2)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title("2R Planar Leg Following Elliptical Trajectory")

line, = ax.plot([], [], 'o-', lw=4, label="Leg")
trace_line, = ax.plot([], [], 'r--', lw=1, label="Foot Trace")
trace_x, trace_y = [], []

def init():
    line.set_data([], [])
    trace_line.set_data([], [])
    return line, trace_line

def update(i):
    # Joint positions
    x0, y0 = 0.0, 0.0
    x1 = L1 * np.cos(theta1[i])
    y1 = L1 * np.sin(theta1[i])
    x2 = x1 + L2 * np.cos(theta1[i] + theta2[i])
    y2 = y1 + L2 * np.sin(theta1[i] + theta2[i])

    line.set_data([x0, x1, x2], [y0, y1, y2])

    # Update foot trace
    trace_x.append(x2)
    trace_y.append(y2)
    trace_line.set_data(trace_x, trace_y)

    return line, trace_line

ani = animation.FuncAnimation(fig, update, frames=len(x_total),
                              init_func=init, interval=20, blit=True)

plt.legend()
plt.show()
