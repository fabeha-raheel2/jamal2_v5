#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import time

def publish_joint_positions():
    rospy.init_node('quadruped_joint_publisher', anonymous=True)

    # Define publishers for each joint's command topic
    joint_publishers = {
        "lh_hip_joint": rospy.Publisher('/jamal2_v5/lh_hip_joint_position_controller/command', Float64, queue_size=10),
        "lh_upper_leg_joint": rospy.Publisher('/jamal2_v5/lh_upper_leg_joint_position_controller/command', Float64, queue_size=10),
        "lh_lower_leg_joint": rospy.Publisher('/jamal2_v5/lh_lower_leg_joint_position_controller/command', Float64, queue_size=10),

        "lf_hip_joint": rospy.Publisher('/jamal2_v5/lf_hip_joint_position_controller/command', Float64, queue_size=10),
        "lf_upper_leg_joint": rospy.Publisher('/jamal2_v5/lf_upper_leg_joint_position_controller/command', Float64, queue_size=10),
        "lf_lower_leg_joint": rospy.Publisher('/jamal2_v5/lf_lower_leg_joint_position_controller/command', Float64, queue_size=10),

        "rh_hip_joint": rospy.Publisher('/jamal2_v5/rh_hip_joint_position_controller/command', Float64, queue_size=10),
        "rh_upper_leg_joint": rospy.Publisher('/jamal2_v5/rh_upper_leg_joint_position_controller/command', Float64, queue_size=10),
        "rh_lower_leg_joint": rospy.Publisher('/jamal2_v5/rh_lower_leg_joint_position_controller/command', Float64, queue_size=10),

        "rf_hip_joint": rospy.Publisher('/jamal2_v5/rf_hip_joint_position_controller/command', Float64, queue_size=10),
        "rf_upper_leg_joint": rospy.Publisher('/jamal2_v5/rf_upper_leg_joint_position_controller/command', Float64, queue_size=10),
        "rf_lower_leg_joint": rospy.Publisher('/jamal2_v5/rf_lower_leg_joint_position_controller/command', Float64, queue_size=10),
    }

    rospy.loginfo("Starting quadruped sit-stand motion...")

    rate = rospy.Rate(10)  # 10 Hz

    # Define joint positions for standing (all-elbow stance)
    stand_positions = {
        "lh_hip_joint": 0.0, "lh_upper_leg_joint": 0.8, "lh_lower_leg_joint": -1.5,
        "lf_hip_joint": 0.0, "lf_upper_leg_joint": 0.8, "lf_lower_leg_joint": -1.5,
        "rh_hip_joint": 0.0, "rh_upper_leg_joint": 0.8, "rh_lower_leg_joint": -1.5,
        "rf_hip_joint": 0.0, "rf_upper_leg_joint": 0.8, "rf_lower_leg_joint": -1.5
    }

    # Define joint positions for sitting
    sit_positions = {
        "lh_hip_joint": 0.0, "lh_upper_leg_joint": 1.5, "lh_lower_leg_joint": -2.5,
        "lf_hip_joint": 0.0, "lf_upper_leg_joint": 1.5, "lf_lower_leg_joint": -2.5,
        "rh_hip_joint": 0.0, "rh_upper_leg_joint": 1.5, "rh_lower_leg_joint": -2.5,
        "rf_hip_joint": 0.0, "rf_upper_leg_joint": 1.5, "rf_lower_leg_joint": -2.5
    }

    while not rospy.is_shutdown():
        # Transition to sitting position
        rospy.loginfo("Moving to sitting position...")
        for i in range(20):  # Smooth transition over 2 seconds
            for joint, pub in joint_publishers.items():
                new_value = stand_positions[joint] + (sit_positions[joint] - stand_positions[joint]) * (i / 20.0)
                pub.publish(Float64(new_value))
            rate.sleep()

        rospy.sleep(2)  # Hold sitting position for 2 seconds

        # Transition to standing position
        rospy.loginfo("Moving to standing position...")
        for i in range(20):  # Smooth transition over 2 seconds
            for joint, pub in joint_publishers.items():
                new_value = sit_positions[joint] + (stand_positions[joint] - sit_positions[joint]) * (i / 20.0)
                pub.publish(Float64(new_value))
            rate.sleep()

        rospy.sleep(2)  # Hold standing position for 2 seconds

if __name__ == '__main__':
    try:
        publish_joint_positions()
    except rospy.ROSInterruptException:
        pass
