#!/usr/bin/env python3
import os
import rospy
import json
import time
from geometry_msgs.msg import Twist, Pose
from champ_msgs.msg import Pose as PoseLite
import tf
from groq import Groq
import re

client = Groq(api_key=os.environ.get("GROQ_API_KEY"))

stop_flag = False  # global interrupt flag

def interpret_command(user_input):
    print("🔹 Sending prompt to Groq...", flush=True)
    prompt = f"""
    You are a control assistant for a quadruped robot.
    Convert the user's instruction into a structured JSON plan with this exact schema:
    {{
        "action": "move" | "rotate" | "pose" | "stop",
        "direction": "forward" | "backward" | "left" | "right" | "rotate_left" | "rotate_right" | "up" | "down" | "none",
        "duration": <seconds or 0 for indefinite>,
        "repeat": true/false
    }}

    The model MUST return a single JSON object (not markdown, not a list, no text before/after).
    If multiple actions are required, return a JSON array instead.
    If the user says "until I tell you to stop", set duration=0 and repeat=true.
    User: "{user_input}"
    """

    try:
        chat_completion = client.chat.completions.create(
            model="llama-3.3-70b-versatile",
            messages=[{"role": "user", "content": prompt}],
        )
        print("✅ Groq API call succeeded.", flush=True)
        response = chat_completion.choices[0].message.content.strip()
        print(f"🧠 Raw LLM response:\n{response}", flush=True)
    except Exception as e:
        print(f"❌ Groq API call failed: {e}", flush=True)
        return {"action": "stop", "direction": "none", "duration": 0, "repeat": False}

    try:
        plan = json.loads(response)
        if isinstance(plan, list):
            plan = plan[0]
    except Exception as e:
        print(f"⚠️ Could not parse cleaned response: {response} ({e})", flush=True)
        plan = {"action": "stop", "direction": "none", "duration": 0, "repeat": False}

    return plan


# The model MUST return a JSON array (only JSON, nothing else). No surrounding text, no markdown, no commentary.

def execute_plan(plan, vel_pub, pose_pub, pose_lite_pub):
    global stop_flag
    stop_flag = False
    print(f"Executing plan: {plan}", flush=True)

    twist = Twist()
    pose_lite = PoseLite()
    pose = Pose()

    # Zero motion
    def stop_motion():
        twist.linear.x = twist.linear.y = twist.angular.z = 0.0
        vel_pub.publish(twist)

    # Set movement
    if plan["action"] in ["move", "rotate"]:
        if plan["direction"] == "forward":
            twist.linear.x = 0.3
        elif plan["direction"] == "backward":
            twist.linear.x = -0.3
        elif plan["direction"] == "left":
            twist.linear.y = 0.3
        elif plan["direction"] == "right":
            twist.linear.y = -0.3
        elif plan["direction"] == "rotate_left":
            twist.angular.z = 0.5
        elif plan["direction"] == "rotate_right":
            twist.angular.z = -0.5

    elif plan["action"] == "pose":
        if plan["direction"] == "down":
            pose_lite.z = -0.2
        elif plan["direction"] == "up":
            pose_lite.z = 0.2
        else:
            pose_lite.z = 0.0

    # Convert RPY → quaternion for full pose
    quaternion = tf.transformations.quaternion_from_euler(
        pose_lite.roll, pose_lite.pitch, pose_lite.yaw
    )
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
    pose.position.z = pose_lite.z

    start_time = time.time()
    rate = rospy.Rate(10)  # 10 Hz publishing

    while not rospy.is_shutdown():
        if stop_flag:
            print("🟥 Stop signal received.", flush=True)
            break

        # Publish continuously
        vel_pub.publish(twist)
        pose_lite_pub.publish(pose_lite)
        pose_pub.publish(pose)

        # Stop after duration if nonzero
        if plan["duration"] > 0 and (time.time() - start_time) > plan["duration"]:
            break

        # For short single poses, publish a few times
        if plan["action"] == "pose" and plan["duration"] == 0:
            rospy.sleep(1.0)
            break

        if not plan["repeat"]:
            break

        rate.sleep()

    stop_motion()
    rospy.loginfo("✅ Motion complete.")


def main():
    global stop_flag
    rospy.init_node("nl_teleop_v2_fixed")

    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    pose_pub = rospy.Publisher("body_pose", Pose, queue_size=1)
    pose_lite_pub = rospy.Publisher("body_pose/raw", PoseLite, queue_size=1)

    rospy.loginfo("Natural Language Teleop v2 (fixed) ready.")

    while not rospy.is_shutdown():
        user_input = input("Command: ").strip()
        if not user_input:
            continue

        if user_input.lower() in ["exit", "quit", "q"]:
            break

        # Interrupt signal
        if "stop" in user_input.lower() or "cancel" in user_input.lower():
            stop_flag = True
            continue

        plan = interpret_command(user_input)
        execute_plan(plan, vel_pub, pose_pub, pose_lite_pub)


if __name__ == "__main__":
    main()
