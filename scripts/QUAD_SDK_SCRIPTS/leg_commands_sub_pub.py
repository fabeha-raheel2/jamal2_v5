#!/usr/bin/env python3
import rospy
from quad_msgs.msg import LegCommandArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def leg_command_callback(msg):
    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()

    # Flatten 4 legs × 3 motors = 12 joints (assuming standard quadruped)
    joint_positions = []
    joint_velocities = []
    joint_efforts = []

    for leg in msg.leg_commands:
        for motor in leg.motor_commands:
            joint_positions.append(motor.pos_setpoint)
            joint_velocities.append(motor.vel_setpoint)
            # joint_efforts.append(motor.torque_ff)
            joint_efforts.append(motor.effort)

    # Fill trajectory message
    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.velocities = joint_velocities
    point.effort = joint_efforts
    point.time_from_start = rospy.Duration(0.01)  # small control interval

    traj_msg.points.append(point)

    # Optional: give joint names if known
    traj_msg.joint_names = [
        f"joint_{i}" for i in range(len(joint_positions))
    ]

    joint_pub.publish(traj_msg)
    rospy.loginfo_throttle(1.0, f"Published JointTrajectory with {len(joint_positions)} joints.")

def main():
    global joint_pub
    rospy.init_node('joint_command_bridge', anonymous=True)

    # Publisher for hardware interface
    joint_pub = rospy.Publisher('/joint_controller/command', JointTrajectory, queue_size=10)

    # Subscriber for high-level leg command array
    rospy.Subscriber('/control/joint_command', LegCommandArray, leg_command_callback)

    rospy.loginfo("Bridging /control/joint_command → /joint_controller/command")
    rospy.spin()

if __name__ == '__main__':
    main()
