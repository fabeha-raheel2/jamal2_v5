#!/usr/bin/env python3
import rospy
import threading
from math import pi, radians
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState, Joy

from Quadruped_config import *
from PcanController import *

class Motor:
    def __init__(self, name='no_name', id=1, min_value=0, max_value=90, offset=0, multiplier=1):
        self.name = name
        self.id = id
        self.min_value = min_value
        self.max_value = max_value
        self.offset = offset
        self.multiplier = multiplier

    def adjust_position(self, pos):
        return self.multiplier * (self.constrain(round(pos, 3)) - self.offset)
    
    def constrain(self, val):
        return max(self.min_value, min(val, self.max_value))
    
    def readjust_position(self, pos):
        return (pos / self.multiplier) + self.offset


class QuadrupedController:
    def __init__(self, publish_joint_state=True, debug=False):
        self._debug = debug
        self.publish_joint_state = publish_joint_state

        # --- Control state flags ---
        self.mode = "idle"        # idle, locomotion, sit, stand
        self.mode_lock = threading.Lock()
        self.is_standing = False  # track if robot is upright
        self.stop_thread = False  # for clean exit

        self.pcan_bus = PcanController()
        self.pcan_bus.initialize()

        self.motors = MOTOR_IDS.copy()
        for motor in MOTOR_IDS.keys():
            self.motors[motor] = Motor(name=motor,
                                       id=MOTOR_IDS[motor],
                                       min_value=radians(MOTOR_MIN_MAX_OFFSET_MULT[motor][0]),
                                       max_value=radians(MOTOR_MIN_MAX_OFFSET_MULT[motor][1]),
                                       offset=radians(MOTOR_MIN_MAX_OFFSET_MULT[motor][2]),
                                       multiplier=MOTOR_MIN_MAX_OFFSET_MULT[motor][3])

        rospy.init_node("Motor_Control_Node")

        # Subscribers
        self.joint_position_subscriber = rospy.Subscriber('/joint_group_position_controller/command', JointTrajectory, self.position_callback)
        self.joystick_subscriber = rospy.Subscriber("/joy", Joy, self.joy_callback)
        
        # Publishers
        if self.publish_joint_state:
            self.joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Safety
        rospy.on_shutdown(self.shutdown_all_motors)


        self.joint_positions = []
        self.feedback_positions = []
        self.joint_names = []
        self.current_positions = CURRENT_POSITIONS.copy()

        # Initialize motors
        for motor in self.motors.values():
            print(motor.name)
            self.pcan_bus.set_motor_origin(motor_id=motor.id)
            self.pcan_bus.enable_motor_mode(motor_id=motor.id)
            self.current_positions[motor.name] = motor.readjust_position(pos=0)
            self.joint_names.append(motor.name)
        # print(self.current_positions)
        # Start the control loop thread
        # self.control_thread = threading.Thread(target=self.run_control_loop, daemon=True)
        # self.control_thread.start()

        rospy.loginfo("QuadrupedController initialized. Waiting for joystick input...")
        # rospy.spin()

    def shutdown_all_motors(self):
        rospy.loginfo("Disabling all motors...")
        for id in MOTOR_IDS.values():
            self.pcan_bus.disable_motor_mode(motor_id=id)
        self.pcan_bus.clean()

    def position_callback(self, msg):
        if msg.points and self.mode == "locomotion":
            self.joint_positions = msg.points[0].positions

    def joy_callback(self, msg):
        sit_button = msg.buttons[6]
        stand_button = msg.buttons[7]

        with self.mode_lock:
            if sit_button == 1 and self.mode != "sit":
                rospy.loginfo("Joystick: SIT command received.")
                self.mode = "sit"
            elif stand_button == 1 and self.mode != "stand":
                rospy.loginfo("Joystick: STAND command received.")
                self.mode = "stand"

    def run_control_loop(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown() and not self.stop_thread:
            with self.mode_lock:
                current_mode = self.mode
                print("Current Mode: ", current_mode)

            if current_mode == "sit":
                rospy.loginfo("Executing sit motion...")
                self.sit()
                with self.mode_lock:
                    self.mode = "idle"
                    self.is_standing = False

            elif current_mode == "stand":
                rospy.loginfo("Executing stand motion...")
                self.stand()
                # print("/////////////////////////////////////")
                with self.mode_lock:
                    self.mode = "locomotion"
                    self.is_standing = True

            elif current_mode == "locomotion":
                # Run main locomotion loop only if robot is standing
                if self.is_standing and self.joint_positions and not self.mode == "sit":
                    self.send_locomotion_commands()

            rate.sleep()

    def send_locomotion_commands(self):
        """Send motor commands based on locomotion controller output"""
        self.feedback_positions = []
        for motor, position in zip(self.motors.values(), self.joint_positions):
            try:
                pos_feedback = self.pcan_bus.send_position(motor_id=motor.id, pos=motor.adjust_position(position))
                self.feedback_positions.append(motor.readjust_position(pos_feedback))
                self.current_positions[motor.name] = motor.readjust_position(pos_feedback)
                # print(self.current_positions)
            except Exception as e:
                rospy.logwarn(f"Error sending motor command: {e}")

        self.publish_joint_states()

    def publish_joint_states(self):
        if not self.publish_joint_state:
            return

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = self.feedback_positions
        self.joint_state_publisher.publish(msg)

    def stand(self):
        rate = rospy.Rate(10)
        start_pos = self.current_positions.copy()
        print("Start Position")
        print(start_pos)
        rospy.loginfo("Moving to stand position...")

        for i in range(30):
            self.feedback_positions = []
            self.joint_names = []
            for motor in self.motors.values():
                new_value = start_pos[motor.name] + ((radians(STAND_TARGETS[motor.name])) - start_pos[motor.name]) * (i / 30.0)
                feedback = self.pcan_bus.send_position(motor_id=motor.id, pos=motor.adjust_position(new_value))
                
                self.feedback_positions.append(motor.readjust_position(feedback))
                self.joint_names.append(motor.name)
                self.current_positions[motor.name] = motor.readjust_position(feedback)
            rate.sleep()

        self.publish_joint_states()
        rospy.loginfo("Standing position reached.")

    def sit(self):
        rate = rospy.Rate(10)
        start_pos = self.current_positions.copy()
        print("Start Position")
        print(start_pos)
        rospy.loginfo("Moving to sit position...")

        for i in range(30):
            self.feedback_positions = []
            self.joint_names = []
            for motor in self.motors.values():
                new_value = start_pos[motor.name] + ((radians(SIT_TARGETS[motor.name])) - start_pos[motor.name]) * (i / 30.0)
                
                feedback = self.pcan_bus.send_position(motor_id=motor.id, pos=motor.adjust_position(new_value))
                self.feedback_positions.append(motor.readjust_position(feedback))
                self.joint_names.append(motor.name)
                self.current_positions[motor.name] = motor.readjust_position(feedback)
            rate.sleep()

        self.publish_joint_states()
        rospy.loginfo("Sitting position reached.")

if __name__ == "__main__":
    quadruped_controller = QuadrupedController()
    quadruped_controller.run_control_loop()