#!/usr/bin/env python3

from __future__ import division
import rospy
from std_msgs.msg import Float64
import time
import math
import numpy as np

### Function to generate elliptical trajectory
def elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, t_range):
    x = x_pre + L * np.cos(np.pi - t_range)
    y = y_pre + H * np.sin(np.pi - t_range)
    z = z_pre + W * np.cos(np.pi - t_range)
    return x, y, z

### Function to generate translation along X-axis
def translation_along_x_axis(x_pre, y_pre, z_pre, L, W, t_range):
    x = np.linspace(x_pre + 2 * L, x_pre, len(t_range))
    y = np.full_like(x, y_pre)
    z = np.linspace(z_pre + 2 * W, z_pre, len(t_range))
    return x, y, z

### Inverse Kinematics Function
def inverse_kinematics(x, y, z, l1, l2, l3):
    if y**2 + z**2 < l1**2:
        rospy.logerr("Invalid leg position: y^2 + z^2 must be >= l1^2.")
        return None

    y_prime = -math.sqrt(y**2 + z**2 - l1**2)
    alpha = math.acos(abs(z) / math.sqrt(y**2 + z**2))
    beta = math.acos(l1 / math.sqrt(y**2 + z**2))

    q1 = alpha - beta if y < 0 and z >= 0 else math.pi - alpha - beta

    phi = math.acos(abs(x) / math.sqrt(x**2 + y_prime**2))
    psi = math.acos((l2**2 + x**2 + y_prime**2 - l3**2) / (2 * l2 * math.sqrt(x**2 + y_prime**2)))

    q3 = math.pi - math.acos((l2**2 + l3**2 - x**2 - y_prime**2) / (2 * l2 * l3))
    q2 = (math.pi / 2) - phi - psi if x > 0 and y_prime < 0 else -(math.pi / 2) + phi - psi

    return q1, q2, -q3

### Function to publish joint commands
def publish_joint_commands(publishers, commands):
    for pub, command in zip(publishers, commands):
        try:
            pub.publish(Float64(command))
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to publish command: {e}")

if __name__ == "__main__":
    rospy.init_node("quadruped_joint_publisher", anonymous=True)

    ### Joint Topics (Modify as per your URDF names)
    joint_names = [
        "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",  
        "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint", 
        "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
        "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
    ]

    ### ROS Publishers
    joint_publishers = [rospy.Publisher(f'/jamal2_v5/{joint}_position_controller/command', Float64, queue_size=10) for joint in joint_names]
    rospy.sleep(1)  # Wait for ROS to initialize publishers

    ### Initial Foot Positions
    x_pre, y_pre, z_pre = 0.0, -0.30, 0.0905  # Rest position
    L, H, W = -0.05, 0.08, 0.00  # Motion parameters
    t_range = np.linspace(0, np.pi, 100)

    rate = rospy.Rate(50)  # 50 Hz control loop

    while not rospy.is_shutdown():
        ### Step 1: Forward Step Motion
        x_traj, y_traj, z_traj = elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, t_range)

        for x, y, z in zip(x_traj, y_traj, z_traj):
            try:
                ### Compute IK for each leg
                q1, q2, q3 = inverse_kinematics(x, y, z, 0.0105, 0.225, 0.230)
                q4, q5, q6 = inverse_kinematics(x, y, z, 0.0105, 0.225, 0.230)
                q7, q8, q9 = inverse_kinematics(x, y, z, 0.0105, 0.225, 0.230)
                qa, qb, qc = inverse_kinematics(x, y, z, 0.0105, 0.225, 0.230)

                if None in [q1, q2, q3, q4, q5, q6, q7, q8, q9, qa, qb, qc]:
                    rospy.logwarn("Skipping invalid IK result.")
                    continue

                joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]
                publish_joint_commands(joint_publishers, joint_commands)

            except ValueError as e:
                rospy.logerr(f"Inverse Kinematics Error: {e}")
                continue

            rate.sleep()

        ### Step 2: Backward Step Motion
        x_traj, y_traj, z_traj = translation_along_x_axis(x_pre, y_pre, z_pre, L, W, t_range)

        for x, y, z in zip(x_traj, y_traj, z_traj):
            try:
                ### Compute IK for each leg
                q1, q2, q3 = inverse_kinematics(x, y, z, 0.0105, 0.225, 0.230)
                q4, q5, q6 = inverse_kinematics(x, y, z, 0.0105, 0.225, 0.230)
                q7, q8, q9 = inverse_kinematics(x, y, z, 0.0105, 0.225, 0.230)
                qa, qb, qc = inverse_kinematics(x, y, z, 0.0105, 0.225, 0.230)

                if None in [q1, q2, q3, q4, q5, q6, q7, q8, q9, qa, qb, qc]:
                    rospy.logwarn("Skipping invalid IK result.")
                    continue

                joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]
                publish_joint_commands(joint_publishers, joint_commands)

            except ValueError as e:
                rospy.logerr(f"Inverse Kinematics Error: {e}")
                continue

            rate.sleep()

        rospy.loginfo("Completed a full gait cycle. Repeating...")

