#!/usr/bin/env python3
import rospy
from quad_msgs.msg import LegCommandArray

def joint_command_callback(msg):
    rospy.loginfo("Received LegCommandArray:")
    for i, leg_cmd in enumerate(msg.leg_commands):
        rospy.loginfo(f"Leg {i}:")
        for j, motor_cmd in enumerate(leg_cmd.motor_commands):
            rospy.loginfo(
                f"  Motor {j} | pos_setpoint: {motor_cmd.pos_setpoint:.3f} | "
                f"vel_setpoint: {motor_cmd.vel_setpoint:.3f} | "
                f"torque_ff: {motor_cmd.torque_ff:.3f} | "
                f"kp: {motor_cmd.kp:.3f} | kd: {motor_cmd.kd:.3f}"
            )

def main():
    rospy.init_node('joint_command_listener', anonymous=True)
    topic_name = '/control/joint_command'  # or check with `rostopic list`
    rospy.Subscriber(topic_name, LegCommandArray, joint_command_callback)

    rospy.loginfo(f"Subscribed to {topic_name}")
    rospy.spin()

if __name__ == '__main__':
    main()
