#!/usr/bin/env python3

from Quadruped import Quadruped
import numpy as np

# INPUTS
L1 = 0.105  # Length of Link 1 (Hip) m
L2 = 0.225  # Length of Link 2 (Thigh) m
L3 = 0.250  # Length of Link 3 (Lower Leg / Shin) m

init_position = [0, -0.45, L1]

robotNameSpace = "jamal2_v5"

x_pre = 0.00     # x-coordinate of end-effector at rest position.
y_pre = -0.45  # y-coordinate of end-effector at rest position.
z_pre = 0.105  # z-coordinate of end-effector at rest position.
L = -0.05   # Decrease to move the end effector forward. Increase to move it backward.
H = 0.08    # Decrease to move the end effector downward. Increase to move it upward.
W = 0.00    # Decrease to move the end effector inward. Increase to move it outward.
t_range = np.linspace(0, np.pi, 100)

jamal2 = Quadruped(ns=robotNameSpace, debug_mode=True)
jamal2.set_link_lengths([L1, L2, L3])

jamal2.move_single_leg_ellipse(leg="rf", x_pre=x_pre, y_pre=y_pre, z_pre=z_pre, L=L, W=W, H=H, t_range=t_range)