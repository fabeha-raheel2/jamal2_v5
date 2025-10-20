#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_POSITIONS = []

def position_callback(msg):

    JOINT_POSITIONS = msg.points[0].positions
    print(JOINT_POSITIONS)

if __name__ == "__main__":
    rospy.init_node("Position_Subscriber")
    joint_position_subscriber = rospy.Subscriber('/joint_group_position_controller/command', JointTrajectory, position_callback)
    rospy.spin()
