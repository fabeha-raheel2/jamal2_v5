#!/usr/bin/env python3

# from __future__ import division
import rospy
from std_msgs.msg import Float64
import time
import math
import numpy as np
import matplotlib.pyplot as plt

a = 1.0
T = 3.0

t_a = None
v = None
def compute_velocity(T, a):
    global t_a, v
    t_a = (a * T) / (2 + a)
    v = a * t_a

compute_velocity(T, a)

def elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, s_func, t_range):

    x = x_pre + L + L * np.cos(np.pi - s_func(t_range))
    y = y_pre + H * np.sin(np.pi - s_func(t_range))
    z = z_pre + W + W * np.cos(np.pi - s_func(t_range))

    return x, y, z

def translation(x_start, x_end, y_start, y_end, z_start, z_end, t_range):
    x = np.linspace(x_start, x_end, len(t_range))
    y = np.linspace(y_start, y_end, len(t_range))
    z = np.linspace(z_start, z_end, len(t_range))

    return x, y, z

def linear_displacement(t):
    return t

def trapezoidal_profile(t):
    global t_a, v, a, T
    t_a_mask1 = t <= t_a
    t_a_mask2 = (t > t_a) & (t <= T - t_a)

    s = np.where(
        t_a_mask1,
        0.5 * a * t**2,
        np.where(
            t_a_mask2,
            v * t - (v**2) / (2 * a),
            (2 * a * v * T - 2 * v**2 - a**2 * (t - T)**2) / (2 * a)
        )
    )

    s_max = s[-1]
    normalized_s = (s / s_max) * np.pi
    return normalized_s

epsilon = 1e-6
l1 = 0.0905  # 9.05 - hip link length
l2 = 0.20    # 20.0 - upper leg link length
l3 = 0.20    # 20.0 - lower leg link length

def inverse_kinematics(x, y, z, l1, l2, l3):
    if y**2 + z**2 < l1**2:
        raise ValueError("Invalid configuration: y^2 + z^2 should be greater than or equal to l1^2.")
    
    y_prime = -math.sqrt(y**2 + z**2 - l1**2)
    
    alpha = math.acos(abs(z) / math.sqrt(y**2 + z**2))
    beta = math.acos(l1 / math.sqrt(y**2 + z**2))
    
    if y < 0 and z >= 0:
        q1 = alpha - beta
    elif y < 0 and z < 0:
        q1 = math.pi - alpha - beta
    else:
        raise ValueError("Invalid y and z configuration for q1 calculation.")
    
    phi = math.acos(abs(x) / math.sqrt(x**2 + y_prime**2))
    psi = math.acos((l2**2 + x**2 + y_prime**2 - l3**2) / (2 * l2 * math.sqrt(x**2 + y_prime**2)))
    
    q3 = -1
    
    if q3 > 0:
        q3 = math.pi - math.acos((l2**2 + l3**2 - x**2 - y_prime**2) / (2 * l2 * l3))
        if x > 0 and y_prime < 0:
            q2 = (math.pi / 2) - phi - psi
        elif x <= 0 and y_prime < 0:
            q2 = -(math.pi / 2) + phi - psi
        else:
            raise ValueError("Invalid x and y' configuration for q2 calculation with positive q3.")
    else:
        q3 = -(math.pi - math.acos((l2**2 + l3**2 - x**2 - y_prime**2) / (2 * l2 * l3)))
        if x >= 0 and y_prime < 0:
            q2 = (math.pi / 2) - phi + psi
        elif x < 0 and y_prime < 0:
            q2 = -(math.pi / 2) + phi + psi
        else:
            raise ValueError("Invalid x and y' configuration for q2 calculation with negative q3.")

    return q1, q2, q3

def publish_joint_commands(publishers, commands):
    for i, pub in enumerate(publishers):
        try:
            pub.publish(commands[i])
        except rospy.ROSException as e:
            rospy.logerr("Failed to publish command to joint {}: {}".format(i, e))

if __name__ == "__main__":
    rospy.init_node('quadruped_joint_publisher', anonymous=True)

    joint_names = [
        "lf_hip_joint_position_controller", "lf_upper_joint_position_controller", "lf_lower_joint_position_controller",  
        "rf_hip_joint_position_controller", "rf_upper_joint_position_controller", "rf_lower_joint_position_controller", 
        "lh_hip_joint_position_controller", "lh_upper_joint_position_controller", "lh_lower_joint_position_controller",
        "rh_hip_joint_position_controller", "rh_upper_joint_position_controller", "rh_lower_joint_position_controller"
    ]

    joint_publishers = []
    for joint in joint_names:
        pub = rospy.Publisher('/jamal2_v5/{}/command'.format(joint), Float64, queue_size=1)
        joint_publishers.append(pub)

    x_pre = 0.00     # x-coordinate of end-effector at rest position.
    y_pre = -0.30   # y-coordinate of end-effector at rest position.
    z_pre = 0.0905  # z-coordinate of end-effector at rest position.
    L = -0.05   # Decrease to move the end effector forward. Increase to move it backward.
    H = 0.08    # Decrease to move the end effector downward. Increase to move it upward.
    W = 0.00    # Decrease to move the end effector inward. Increase to move it outward.
    t_range = np.linspace(0, np.pi, 100)

    rate = rospy.Rate(100)

    for i in range(50):

        start_time = time.time()

        try:
            qa, qb, qc = q7, q8, q9 = q4, q5, q6 = q1, q2, q3 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
        except ValueError as e:
            rospy.logerr(e)
            continue

        joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]

        rospy.loginfo("Publishing joint commands: {}".format(joint_commands))
        publish_joint_commands(joint_publishers, joint_commands)

        end_time = time.time()

        iteration_duration = end_time - start_time
        rospy.loginfo("Iteration {} took {} seconds".format(i, iteration_duration))
                
        rate.sleep()

    time.sleep(3)

    while not rospy.is_shutdown():
    
        x_e, y_e, z_e = elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, trapezoidal_profile, t_range)

        for i in range(len(x_e)):
            curr_x_e = x_e[i]
            curr_y_e = y_e[i]
            curr_z_e = z_e[i]

            rospy.loginfo("Current trajectory values: x_e={}, y_e={}, z_e={}".format(curr_x_e, curr_y_e, curr_z_e))

            try:
                q1, q2, q3 = inverse_kinematics(curr_x_e, curr_y_e, curr_z_e, l1, l2, l3)
                q4, q5, q6 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                q7, q8, q9 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                qa, qb, qc = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
            except ValueError as e:
                rospy.logerr(e)
                continue

            joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]

            rospy.loginfo("Publishing joint commands: {}".format(joint_commands))
            publish_joint_commands(joint_publishers, joint_commands)
                
            rate.sleep()

        x_e, y_e, z_e = elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, trapezoidal_profile, t_range)
        # x_t, y_t, z_t = translate_backward(x_pre - L, y_pre, z_pre, L, W, t_range)
        # x_t, y_t, z_t = translate_backward(x_pre, y_pre, z_pre, L+L , W, t_range)

        for i in range(len(x_e)):
            curr_x_e = x_e[i]
            curr_y_e = y_e[i]
            curr_z_e = z_e[i]

            # curr_x_t = x_t[i]
            # curr_y_t = y_t[i]
            # curr_z_t = z_t[i]

            rospy.loginfo("Current trajectory values: x_e={}, y_e={}, z_e={}".format(curr_x_e, curr_y_e, curr_z_e))
            # rospy.loginfo("Current trajectory values: x_t={}, y_t={}, z_t={}".format(curr_x_t, curr_y_t, curr_z_t))

            try:
                # q1, q2, q3 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                q4, q5, q6 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                q7, q8, q9 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                qa, qb, qc = inverse_kinematics(curr_x_e, curr_y_e, curr_z_e, l1, l2, l3)
            except ValueError as e:
                rospy.logerr(e)
                continue

            joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]

            rospy.loginfo("Publishing joint commands: {}".format(joint_commands))
            publish_joint_commands(joint_publishers, joint_commands)
                
            rate.sleep()

        # x_e_out, y_e, z_e = elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, linear_displacement, t_range)
        x_t_out, y_t, z_t = translation(L*2, x_pre, y_pre, y_pre, z_pre, z_pre, t_range) # -0.1, 0.0
        x_t_out_l, y_t, z_t = translation(x_pre, -L*2, y_pre, y_pre, z_pre, z_pre, t_range)# 0.0, 0.1

        for i in range(len(x_e)):
            curr_x_t_out = x_t_out[i]
            curr_x_t_out_l = x_t_out_l[i]
            curr_y_t = y_t[i]
            curr_z_t = z_t[i]

            # rospy.loginfo("Current trajectory values: x_e_out={}, x_e_out={}, y_e={}, z_e={}".format(curr_x_e_out, curr_x_e_out, curr_y_e, curr_z_e))
            rospy.loginfo("Current trajectory values: x_t_out={}, x_e_out={}, y_t={}, z_t={}".format(curr_x_t_out, curr_x_t_out, curr_y_t, curr_z_t))

            try:
                q1, q2, q3 = inverse_kinematics(curr_x_t_out, curr_y_t, curr_z_t, l1, l2, l3)
                q4, q5, q6 = inverse_kinematics(curr_x_t_out_l, curr_y_t, curr_z_t, l1, l2, l3)
                q7, q8, q9 = inverse_kinematics(curr_x_t_out_l, curr_y_t, curr_z_t, l1, l2, l3)
                qa, qb, qc = inverse_kinematics(curr_x_t_out, curr_y_t, curr_z_t, l1, l2, l3)
            except ValueError as e:
                rospy.logerr(e)
                continue

            joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]

            rospy.loginfo("Publishing joint commands: {}".format(joint_commands))
            publish_joint_commands(joint_publishers, joint_commands)
                
            rate.sleep()

    # repeating for other diagonal foots

        x_e, y_e, z_e = elliptical_trajectory(-L*2 , y_pre, z_pre, L, H, W, trapezoidal_profile, t_range)
        # x_t_out, y_t, z_t = translate_backward(x_pre, y_pre, z_pre, L, W, t_range)

        for i in range(len(x_e)):
            curr_x_e = x_e[i]
            curr_y_e = y_e[i]
            curr_z_e = z_e[i]

            # curr_x_t_out = x_t_out[i]
            # curr_y_t = y_t[i]
            # curr_z_t = z_t[i]

            rospy.loginfo("Current trajectory values: x_e={}, y_e={}, z_e={}".format(curr_x_e, curr_y_e, curr_z_e))
            # rospy.loginfo("Current trajectory values: x_t_out={}, x_e_out={}, y_t={}, z_t={}".format(curr_x_t_out, curr_x_t_out, curr_y_t, curr_z_t))

            try:
                q1, q2, q3 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                q4, q5, q6 = inverse_kinematics(curr_x_e, curr_y_e, curr_z_e, l1, l2, l3)
                # q7, q8, q9 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                qa, qb, qc = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
            except ValueError as e:
                rospy.logerr(e)
                continue

            joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]

            rospy.loginfo("Publishing joint commands: {}".format(joint_commands))
            publish_joint_commands(joint_publishers, joint_commands)
                
            rate.sleep()

        x_e, y_e, z_e = elliptical_trajectory(-L*2, y_pre, z_pre, L, H, W, trapezoidal_profile, t_range)
        # x_t_out, y_t, z_t = translate_backward(x_pre-L, y_pre, z_pre, L, W, t_range)
        # x_t_out_l, y_t, z_t = translate_backward(x_pre, y_pre, z_pre, L+L , W, t_range)

        for i in range(len(x_e)):
            curr_x_e = x_e[i]
            curr_y_e = y_e[i]
            curr_z_e = z_e[i]

            # curr_x_t_out = x_t_out[i]
            # curr_x_t_out_l = x_t_out_l[i]
            # curr_y_t = y_t[i]
            # curr_z_t = z_t[i]

            rospy.loginfo("Current trajectory values: x_e={}, y_e={}, z_e={}".format(curr_x_e, curr_y_e, curr_z_e))
            # rospy.loginfo("Current trajectory values: x_t_out={}, x_e_out={}, y_t={}, z_t={}".format(curr_x_t_out, curr_x_e_out, curr_y_t, curr_z_t))

            try:
                q1, q2, q3 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                # q4, q5, q6 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                q7, q8, q9 = inverse_kinematics(curr_x_e, curr_y_e, curr_z_e, l1, l2, l3)
                qa, qb, qc = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
            except ValueError as e:
                rospy.logerr(e)
                continue

            joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]

            rospy.loginfo("Publishing joint commands: {}".format(joint_commands))
            publish_joint_commands(joint_publishers, joint_commands)
                
            rate.sleep()

