import rospy
import math
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

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
    
    def __str__(self):
        return f"Motor {self.name} with ID {self.id}"
    
    def __repr__(self):
        return f"Motor {self.name} with ID {self.id}"

class JamalController:
    def __init__(self, publish_feedback=True, debug=True):
        self._debug = debug

        self.pcan_bus = PcanController()

        self.publish_joint_state = publish_feedback

        self.joint_states = {"positions":[], "velocities":[], "torques":[]} # feedback that is received from motors
        self.joint_commands = {"positions":[], "velocities":[], "torques":[], "kp":[], "kd":[]} # the commands that have to be sent to the motors
        
        self.joint_names = JOINT_NAMES

        self.motors = MOTOR_IDS.copy()

        # TO-DO: Do we still need this?
        for motor in MOTOR_IDS.keys():
            self.motors[motor] = Motor(name=motor,
                                       id=MOTOR_IDS[motor],
                                       min_value=math.radians(MOTOR_MIN_MAX_OFFSET_MULT[motor][0]),
                                       max_value=math.radians(MOTOR_MIN_MAX_OFFSET_MULT[motor][1]),
                                       offset=math.radians(MOTOR_MIN_MAX_OFFSET_MULT[motor][2]),
                                       multiplier=MOTOR_MIN_MAX_OFFSET_MULT[motor][3])

        # Jamal Motor Control Node

        rospy.init_node("Motor_Control_Node")
        self.joint_position_subscriber = rospy.Subscriber('/joint_controller/command', Float64MultiArray, self.controller_callback)
        
        if self.publish_joint_state:
            self.joint_state_publisher = rospy.Publisher('/joint_states2', JointState, queue_size=10)

        self.pcan_bus.initialize()

        self.feedback_positions = []
        if self.publish_joint_state:
            if self._debug:
                user_input = input("Set Motors 0 Position?")

                if user_input == "y" or user_input == "Y":
                    for motor in self.motors.values():
                        self.pcan_bus.set_motor_origin(motor_id=motor.id)
                        self.pcan_bus.enable_motor_mode(motor_id=motor.id)
                        self.pcan_bus.send_position(motor_id=motor.id, pos=0)
                        self.feedback_positions.append(motor.readjust_position(pos=0))
            # else:
            #     self.feedback_positions.append(motor.readjust_position(pos=0))

        if self.publish_joint_state:
                # self.publish_joint_feedback()
                msg = JointState()

                msg.header.stamp = rospy.Time.now()
                msg.name = JOINT_NAMES
                msg.position = self.feedback_positions

                self.joint_state_publisher.publish(msg)

    def controller_callback(self, msg):
        self.joint_commands = {"positions":[], "velocities":[], "torques":[0]*12, "kp":[0]*12, "kd":[0]*12}

        # Get the commands from the NMPC + WBC controller
        self.joint_commands["positions"] = msg.data[0:12]
        self.joint_commands["velocities"] = msg.data[12:24]
        # self.joint_commands["kp"] = msg.data[24:36]
        # self.joint_commands["kd"] = msg.data[36:48]
        # self.joint_commands["torques"] = msg.data[48:60]
        # print(msg.data)

        # print("Joint Commands: ", self.joint_commands)

        # Send these commands to each of the motors
        self.send_motor_commands()

        # Publish the feedback of all 12 motors
        if self.publish_joint_state:
            self.publish_joint_feedback()
            
    def send_motor_commands(self):
        self.joint_states = {"positions":[], "velocities":[], "torques":[]}

        for motor, position, velocity, torque, kp, kd in zip(self.motors.values(), self.joint_commands["positions"], self.joint_commands["velocities"], self.joint_commands["torques"], self.joint_commands["kp"], self.joint_commands["kd"]):

            # Send the command    
            try:
                feedback = self.pcan_bus.send_motor_data(motor_id=motor.id, 
                                                            pos=motor.adjust_position(position), 
                                                            v_in=velocity,
                                                            t_in=torque,
                                                            kp_in=kp,
                                                            kd_in=kd)
                
            except KeyboardInterrupt:
                print("\nDisabling motor and exiting...")
                
                for id in MOTOR_IDS.values():
                    self.pcan_bus.disable_motor_mode(motor_id=id)
                
                self.pcan_bus.clean()
                break

            # Save the feedback
            self.joint_states['positions'].append(motor.readjust_position(feedback['position']))
            self.joint_states['velocities'].append(feedback['velocity'])
            self.joint_states['torques'].append(feedback['torque'])

    def publish_joint_feedback(self):
        msg = JointState()

        msg.header.stamp = rospy.Time.now()
        msg.name = JOINT_NAMES
        msg.position = self.joint_states['positions']
        msg.velocity = self.joint_states['velocities']
        msg.effort = self.joint_states['torques']
        print(self.joint_states)
        self.joint_state_publisher.publish(msg)

            
              
if __name__ == "__main__":
    quadruped_controller = JamalController()
    rospy.spin()
