#!/usr/bin/env python
import os
import rospy
from geometry_msgs.msg import Twist, Pose
from champ_msgs.msg import Pose as PoseLite
import tf
from groq import Groq

# Initialize the Groq client
client = Groq(api_key=os.environ.get("GROQ_API_KEY"))

def interpret_command(user_input):
    """
    Use the LLM to convert a natural-language command
    into a structured robot action.
    """
    prompt = f"""
    You are a control assistant for a quadruped robot.
    Convert the user's natural-language instruction into one of these simple commands:
    [forward, backward, left, right, rotate_left, rotate_right, sit, stand, stop, up, down]
    Only return the command word, nothing else.

    User: "{user_input}"
    """
    chat_completion = client.chat.completions.create(
        model="llama-3.3-70b-versatile",
        messages=[{"role": "user", "content": prompt}],
    )
    return chat_completion.choices[0].message.content.strip().lower()

def execute_command(cmd, vel_pub, pose_pub, pose_lite_pub):
    twist = Twist()
    pose_lite = PoseLite()
    pose = Pose()

    # Set default stop values
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.angular.z = 0.0
    pose_lite.roll = 0.0
    pose_lite.pitch = 0.0
    pose_lite.yaw = 0.0
    pose_lite.z = 0.0

    # Map commands to actions
    if cmd == "forward":
        twist.linear.x = 0.3
    elif cmd == "backward":
        twist.linear.x = -0.3
    elif cmd == "left":
        twist.linear.y = 0.3
    elif cmd == "right":
        twist.linear.y = -0.3
    elif cmd == "rotate_left":
        twist.angular.z = 0.5
    elif cmd == "rotate_right":
        twist.angular.z = -0.5
    elif cmd == "sit":
        pose_lite.z = -0.2
    elif cmd == "stand":
        pose_lite.z = 0.0
    elif cmd == "up":
        pose_lite.z = 0.1
    elif cmd == "down":
        pose_lite.z = -0.1
    elif cmd == "stop":
        pass
    else:
        rospy.logwarn(f"Unknown command: {cmd}")

    # Publish movement
    vel_pub.publish(twist)

    # Convert roll/pitch/yaw to quaternion for Pose
    quaternion = tf.transformations.quaternion_from_euler(
        pose_lite.roll, pose_lite.pitch, pose_lite.yaw
    )
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    pose.position.z = pose_lite.z

    pose_lite_pub.publish(pose_lite)
    pose_pub.publish(pose)

def main():
    rospy.init_node("nl_teleop")

    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    pose_pub = rospy.Publisher("body_pose", Pose, queue_size=1)
    pose_lite_pub = rospy.Publisher("body_pose/raw", PoseLite, queue_size=1)

    rospy.loginfo("Natural Language Teleop ready. Type commands (e.g., 'move forward', 'sit down').")

    while not rospy.is_shutdown():
        user_input = input("Command: ").strip()
        if not user_input:
            continue
        if user_input.lower() in ["exit", "quit", "q"]:
            break

        rospy.loginfo(f"Interpreting: {user_input}")
        cmd = interpret_command(user_input)
        rospy.loginfo(f"→ LLM interpreted command: {cmd}")
        execute_command(cmd, vel_pub, pose_pub, pose_lite_pub)

if __name__ == "__main__":
    main()
