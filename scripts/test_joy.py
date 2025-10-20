#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy

stand_flag = False
sit_flag = True

def joy_callback(msg):
    # rospy.loginfo(f"Axes: {msg.axes}")
    # rospy.loginfo(f"Buttons: {msg.buttons}")
    sit = msg.buttons[6]
    stand = msg.buttons[7]

    if sit == 1:
        # sit_flag = True
        # stand_flag = False
        rospy.loginfo("Sit command received")

    elif stand == 1:
        # stand_flag = True
        # sit_flag = False
        rospy.loginfo("Stand command received")

    # else:
        # global sit_flag
        # sit_flag = True

def listener():
    rospy.init_node('joystick_subscriber', anonymous=True)
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
