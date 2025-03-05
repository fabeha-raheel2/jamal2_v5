#!/usr/bin/env python3

from __future__ import division
import rospy
from std_msgs.msg import Float64
import time
import math
import numpy as np
import matplotlib.pyplot as plt

def elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, s_func, t_range):
    x = x_pre + L + L * np.cos(np.pi - s_func(t_range))
    y = y_pre + H * np.sin(np.pi - s_func(t_range))
    z = z_pre + W + W * np.cos(np.pi - s_func(t_range))

    return x, y, z

def translation_along_x_axis(x_pre, y_pre, z_pre, L, W, t_range):
    x = np.linspace((x_pre + 2 * L), x_pre, len(t_range))
    y = np.full_like(x, y_pre)
    z = np.linspace((z_pre + 2 * W), z_pre, len(t_range))

    return x, y, z

def linear_displacement(t):
    return t

epsilon = 1e-6
l1 = 0.0105  # 9.05 - hip link length
l2 = 0.225    # 20.0 - upper leg link length
l3 = 0.230    # 20.0 - lower leg link length

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
        pub = rospy.Publisher('/jamal_v2/{}/command'.format(joint), Float64, queue_size=1)
        joint_publishers.append(pub)

    x_pre = 0.0     # x-coordinate of end-effector at rest position.
    y_pre = -0.30   # y-coordinate of end-effector at rest position.
    z_pre = 0.0905  # z-coordinate of end-effector at rest position.
    L_outward = -0.05    # Decrease to move the end effector forward. Increase to move it backward.
    L_inward = -0.05
    L = -0.05
    H = 0.08    # Decrease to move the end effector downward. Increase to move it upward.
    W = 0.00    # Decrease to move the end effector inward. Increase to move it outward.
    t_range = np.linspace(0, np.pi, 100)

    rate = rospy.Rate(100)


    q1, q2, q3 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
    q4, q5, q6 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
    q7, q8, q9 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
    qa, qb, qc = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)

    joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]

    rospy.loginfo("Publishing joint commands: {}".format(joint_commands))
    publish_joint_commands(joint_publishers, joint_commands)
    time.sleep(3)
    rate.sleep()
    while not rospy.is_shutdown():
    
        x_e_out, y_e, z_e = elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, linear_displacement, t_range)
        x_t_out, y_t, z_t = translation_along_x_axis(x_pre, y_pre, z_pre, L, W, t_range)

        for i in range(len(x_e_out)):
            curr_x_e_out = x_e_out[i]
            curr_y_e = y_e[i]
            curr_z_e = z_e[i]

            curr_x_t_out = x_t_out[i]
            curr_y_t = y_t[i]
            curr_z_t = z_t[i]

            rospy.loginfo("Current trajectory values: x_e_out={}, x_e_out={}, y_e={}, z_e={}".format(curr_x_e_out, curr_x_e_out, curr_y_e, curr_z_e))
            rospy.loginfo("Current trajectory values: x_t_out={}, x_e_out={}, y_t={}, z_t={}".format(curr_x_t_out, curr_x_e_out, curr_y_t, curr_z_t))

            try:
                q1, q2, q3 = inverse_kinematics(curr_x_e_out, curr_y_e, curr_z_e, l1, l2, l3)
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

        x_e_out, y_e, z_e = elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, linear_displacement, t_range)
        x_t_out, y_t, z_t = translation_along_x_axis(x_pre - L, y_pre, z_pre, L, W, t_range)
        x_t_out_l, y_t, z_t = translation_along_x_axis(x_pre, y_pre, z_pre, L+L , W, t_range)

        for i in range(len(x_e_out)):
            curr_x_e_out = x_e_out[i]
            curr_y_e = y_e[i]
            curr_z_e = z_e[i]

            curr_x_t_out = x_t_out[i]
            curr_x_t_out_l = x_t_out_l[i]
            curr_y_t = y_t[i]
            curr_z_t = z_t[i]

            rospy.loginfo("Current trajectory values: x_e_out={}, x_e_out={}, y_e={}, z_e={}".format(curr_x_e_out, curr_x_e_out, curr_y_e, curr_z_e))
            rospy.loginfo("Current trajectory values: x_t_out={}, x_e_out={}, y_t={}, z_t={}".format(curr_x_t_out, curr_x_e_out, curr_y_t, curr_z_t))

            try:
                # q1, q2, q3 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                q4, q5, q6 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                q7, q8, q9 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                qa, qb, qc = inverse_kinematics(curr_x_e_out, curr_y_e, curr_z_e, l1, l2, l3)
            except ValueError as e:
                rospy.logerr(e)
                continue

            joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]

            rospy.loginfo("Publishing joint commands: {}".format(joint_commands))
            publish_joint_commands(joint_publishers, joint_commands)
                
            rate.sleep()

        # x_e_out, y_e, z_e = elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, linear_displacement, t_range)
        x_t_out, y_t, z_t = translation_along_x_axis(x_pre, y_pre, z_pre, L, W, t_range)
        x_t_out_l, y_t, z_t = translation_along_x_axis(-L*2, y_pre, z_pre, L , W, t_range)

        for i in range(len(x_e_out)):
            curr_x_e_out = x_e_out[i]
            curr_y_e = y_e[i]
            curr_z_e = z_e[i]

            curr_x_t_out = x_t_out[i]
            curr_x_t_out_l = x_t_out_l[i]
            curr_y_t = y_t[i]
            curr_z_t = z_t[i]

            rospy.loginfo("Current trajectory values: x_e_out={}, x_e_out={}, y_e={}, z_e={}".format(curr_x_e_out, curr_x_e_out, curr_y_e, curr_z_e))
            rospy.loginfo("Current trajectory values: x_t_out={}, x_e_out={}, y_t={}, z_t={}".format(curr_x_t_out, curr_x_e_out, curr_y_t, curr_z_t))

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

        x_e_out, y_e, z_e = elliptical_trajectory(-L*2 , y_pre, z_pre, L, H, W, linear_displacement, t_range)
        x_t_out, y_t, z_t = translation_along_x_axis(x_pre, y_pre, z_pre, L, W, t_range)

        for i in range(len(x_e_out)):
            curr_x_e_out = x_e_out[i]
            curr_y_e = y_e[i]
            curr_z_e = z_e[i]

            curr_x_t_out = x_t_out[i]
            curr_y_t = y_t[i]
            curr_z_t = z_t[i]

            rospy.loginfo("Current trajectory values: x_e_out={}, x_e_out={}, y_e={}, z_e={}".format(curr_x_e_out, curr_x_e_out, curr_y_e, curr_z_e))
            rospy.loginfo("Current trajectory values: x_t_out={}, x_e_out={}, y_t={}, z_t={}".format(curr_x_t_out, curr_x_e_out, curr_y_t, curr_z_t))

            try:
                q1, q2, q3 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                q4, q5, q6 = inverse_kinematics(curr_x_e_out, curr_y_e, curr_z_e, l1, l2, l3)
                # q7, q8, q9 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                qa, qb, qc = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
            except ValueError as e:
                rospy.logerr(e)
                continue

            joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]

            rospy.loginfo("Publishing joint commands: {}".format(joint_commands))
            publish_joint_commands(joint_publishers, joint_commands)
                
            rate.sleep()

        x_e_out, y_e, z_e = elliptical_trajectory(-L*2, y_pre, z_pre, L, H, W, linear_displacement, t_range)
        x_t_out, y_t, z_t = translation_along_x_axis(x_pre-L_inward, y_pre, z_pre, L, W, t_range)
        x_t_out_l, y_t, z_t = translation_along_x_axis(x_pre, y_pre, z_pre, L+L , W, t_range)

        for i in range(len(x_e_out)):
            curr_x_e_out = x_e_out[i]
            curr_y_e = y_e[i]
            curr_z_e = z_e[i]

            curr_x_t_out = x_t_out[i]
            curr_x_t_out_l = x_t_out_l[i]
            curr_y_t = y_t[i]
            curr_z_t = z_t[i]

            rospy.loginfo("Current trajectory values: x_e_out={}, x_e_out={}, y_e={}, z_e={}".format(curr_x_e_out, curr_x_e_out, curr_y_e, curr_z_e))
            rospy.loginfo("Current trajectory values: x_t_out={}, x_e_out={}, y_t={}, z_t={}".format(curr_x_t_out, curr_x_e_out, curr_y_t, curr_z_t))

            try:
                q1, q2, q3 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                # q4, q5, q6 = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
                q7, q8, q9 = inverse_kinematics(curr_x_e_out, curr_y_e, curr_z_e, l1, l2, l3)
                qa, qb, qc = inverse_kinematics(x_pre, y_pre, z_pre, l1, l2, l3)
            except ValueError as e:
                rospy.logerr(e)
                continue

            joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]

            rospy.loginfo("Publishing joint commands: {}".format(joint_commands))
            publish_joint_commands(joint_publishers, joint_commands)
                
            rate.sleep()

        # x_e_out, y_e, z_e = elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, linear_displacement, t_range)
        # x_t_out, y_t, z_t = translation_along_x_axis(x_pre, y_pre, z_pre, L, W, t_range)
        # x_t_out_l, y_t, z_t = translation_along_x_axis(x_pre-L, y_pre, z_pre, L , W, t_range)

        # for i in range(len(x_e_out)):
        #     curr_x_e_out = x_e_out[i]
        #     curr_y_e = y_e[i]
        #     curr_z_e = z_e[i]

        #     curr_x_t_out = x_t_out[i]
        #     curr_x_t_out_l = x_t_out_l[i]
        #     curr_y_t = y_t[i]
        #     curr_z_t = z_t[i]

        #     rospy.loginfo("Current trajectory values: x_e_out={}, x_e_out={}, y_e={}, z_e={}".format(curr_x_e_out, curr_x_e_out, curr_y_e, curr_z_e))
        #     rospy.loginfo("Current trajectory values: x_t_out={}, x_e_out={}, y_t={}, z_t={}".format(curr_x_t_out, curr_x_e_out, curr_y_t, curr_z_t))

        #     try:
        #         q1, q2, q3 = inverse_kinematics(curr_x_t_out_l, curr_y_t, curr_z_t, l1, l2, l3)
        #         q4, q5, q6 = inverse_kinematics(curr_x_t_out, curr_y_t, curr_z_t, l1, l2, l3)
        #         q7, q8, q9 = inverse_kinematics(curr_x_t_out, curr_y_t, curr_z_t, l1, l2, l3)
        #         qa, qb, qc = inverse_kinematics(curr_x_t_out_l, curr_y_t, curr_z_t, l1, l2, l3)
        #     except ValueError as e:
        #         rospy.logerr(e)
        #         continue

        #     joint_commands = [-q1, q2, q3, q4, q5, q6, -q7, q8, q9, qa, qb, qc]

        #     rospy.loginfo("Publishing joint commands: {}".format(joint_commands))
        #     publish_joint_commands(joint_publishers, joint_commands)
                
        #     rate.sleep()

