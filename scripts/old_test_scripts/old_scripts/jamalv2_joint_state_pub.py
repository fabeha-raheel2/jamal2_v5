#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def publish_joint_positions():
    rospy.init_node('quadruped_joint_publisher', anonymous=True)

    # Define publishers for each joint's command topic
    joint_publishers = {
        "lh_hip_joint": rospy.Publisher('/jamal_v2/lh_hip_joint_position_controller/command', Float64, queue_size=10),
        "lh_upper_leg_joint": rospy.Publisher('/jamal_v2/lh_upper_leg_joint_position_controller/command', Float64, queue_size=10),
        "lh_lower_leg_joint": rospy.Publisher('/jamal_v2/lh_lower_leg_joint_position_controller/command', Float64, queue_size=10),

        "lf_hip_joint": rospy.Publisher('/jamal_v2/lf_hip_joint_position_controller/command', Float64, queue_size=10),
        "lf_upper_leg_joint": rospy.Publisher('/jamal_v2/lf_upper_leg_joint_position_controller/command', Float64, queue_size=10),
        "lf_lower_leg_joint": rospy.Publisher('/jamal_v2/lf_lower_leg_joint_position_controller/command', Float64, queue_size=10),

        "rh_hip_joint": rospy.Publisher('/jamal_v2/rh_hip_joint_position_controller/command', Float64, queue_size=10),
        "rh_upper_leg_joint": rospy.Publisher('/jamal_v2/rh_upper_leg_joint_position_controller/command', Float64, queue_size=10),
        "rh_lower_leg_joint": rospy.Publisher('/jamal_v2/rh_lower_leg_joint_position_controller/command', Float64, queue_size=10),

        "rf_hip_joint": rospy.Publisher('/jamal_v2/rf_hip_joint_position_controller/command', Float64, queue_size=10),
        "rf_upper_leg_joint": rospy.Publisher('/jamal_v2/rf_upper_leg_joint_position_controller/command', Float64, queue_size=10),
        "rf_lower_leg_joint": rospy.Publisher('/jamal_v2/rf_lower_leg_joint_position_controller/command', Float64, queue_size=10),
    }

    # Estimated stance angles (in radians) for all-elbow configuration
    joint_positions = {
        "lh_hip_joint": 0.0, "lh_upper_leg_joint": 0.8, "lh_lower_leg_joint": -1.5,
        "lf_hip_joint": 0.0, "lf_upper_leg_joint": 0.8, "lf_lower_leg_joint": -1.5,
        "rh_hip_joint": 0.0, "rh_upper_leg_joint": 0.8, "rh_lower_leg_joint": -1.5,
        "rf_hip_joint": 0.0, "rf_upper_leg_joint": 0.8, "rf_lower_leg_joint": -1.5
    }

    rospy.loginfo("Publishing joint positions for stance in all-elbow configuration...")
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        for joint, pub in joint_publishers.items():
            pub.publish(Float64(joint_positions[joint]))  # Send the command
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_positions()
    except rospy.ROSInterruptException:
        pass
