#!/usr/bin/env python3
import rospy
from quad_msgs.msg import LegCommandArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Define your joint name sequence (must match the order of data extraction)
JOINT_NAMES = [
    '8', '0', '1',   # Left Front
    '9', '2', '3',   # Left Hind
    '10', '4', '5',  # Right Front
    '11', '6', '7'   # Right Hind
]

def leg_command_callback(msg):
    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = JOINT_NAMES

    joint_positions = []
    joint_velocities = []
    joint_efforts = []

    # Flatten all leg commands (4 legs × 3 motors)
    for leg in msg.leg_commands:
        for motor in leg.motor_commands:
            joint_positions.append(motor.pos_setpoint)
            joint_velocities.append(motor.vel_setpoint)
            joint_efforts.append(motor.torque_ff)
            # joint_efforts.append(motor.effort)

    # Make sure we have the correct number of joints
    if len(joint_positions) != len(JOINT_NAMES):
        rospy.logwarn(f"Joint count mismatch: got {len(joint_positions)}, expected {len(JOINT_NAMES)}")
        return

    # Fill in the trajectory point
    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.velocities = joint_velocities
    point.effort = joint_efforts
    point.time_from_start = rospy.Duration(0.01)

    traj_msg.points.append(point)

    # Publish trajectory
    joint_pub.publish(traj_msg)
    rospy.loginfo_throttle(1.0, f"Published JointTrajectory with {len(joint_positions)} joints.")

def main():
    global joint_pub
    rospy.init_node('joint_command_bridge', anonymous=True)

    joint_pub = rospy.Publisher('/joint_controller/command', JointTrajectory, queue_size=10)
    rospy.Subscriber('/control/joint_command', LegCommandArray, leg_command_callback)

    rospy.loginfo("Bridging /control/joint_command → /joint_controller/command")
    rospy.spin()

if __name__ == '__main__':
    main()
