#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time

def publish_vel(pub, lin_x=0.0, lin_y=0.0, lin_z=0.0, ang_x=0.0, ang_y=0.0, ang_z=0.0):
    
    # Create a Twist message
    vel_msg = Twist()

    # Example: Move forward with 0.5 m/s and rotate at 0.2 rad/s
    vel_msg.linear.x = lin_x
    vel_msg.linear.y = lin_y
    vel_msg.linear.z = lin_z
    vel_msg.angular.x = ang_x
    vel_msg.angular.y = ang_y
    vel_msg.angular.z = ang_z

    # rospy.loginfo("Publishing to /cmd_vel...")
    print("Publishing to /cmd_vel...")
    pub.publish(vel_msg)

def main():
    # Initialize ROS node
    rospy.init_node('cmd_vel_publisher', anonymous=True)

    # Create publisher for /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    publish_vel(pub, lin_z=0.01)
    time.sleep(0.5)
    publish_vel(pub,lin_z=0.0)
    time.sleep(0.2)

    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
