#!/usr/bin/env python3

from Quadruped import Quadruped

# INPUTS
L1 = 0.105  # Length of Link 1 (Hip) m
L2 = 0.225  # Length of Link 2 (Thigh) m
L3 = 0.250  # Length of Link 3 (Lower Leg / Shin) m

init_position = [0, -0.45, L1]

robotNameSpace = "jamal2_v5"

jamal2 = Quadruped(ns=robotNameSpace, debug_mode=True)
jamal2.set_link_lengths([L1, L2, L3])

#### TEST LEG IK ...................................................

joint_angles = jamal2.compute_inv_kinematics(position=init_position)

jamal2.publish_leg_commands(leg="rf", joint_angles=joint_angles)