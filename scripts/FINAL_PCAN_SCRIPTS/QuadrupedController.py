import rospy
import math
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

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

class QuadrupedController:
    def __init__(self, publish_joint_state=False):
        self.pcan_bus = PcanController()

        self.publish_joint_state = publish_joint_state

        self.joint_positions = []
        self.joint_names = JOINT_NAMES

        self.motors = MOTOR_IDS.copy()

        for motor in MOTOR_IDS.keys():
            self.motors[motor] = Motor(name=motor,
                                       id=MOTOR_IDS[motor],
                                       min_value=math.radians(MOTOR_MIN_MAX_OFFSET_MULT[motor][0]),
                                       max_value=math.radians(MOTOR_MIN_MAX_OFFSET_MULT[motor][1]),
                                       offset=math.radians(MOTOR_MIN_MAX_OFFSET_MULT[motor][2]),
                                       multiplier=MOTOR_MIN_MAX_OFFSET_MULT[motor][3])

        rospy.init_node("Motor_Control_Node")
        self.joint_position_subscriber = rospy.Subscriber('/joint_group_position_controller/command', JointTrajectory, self.position_callback)
        
        if self.publish_joint_state:
            self.joint_state_publisher = rospy.Publisher('/joint_states', JointState)

        self.pcan_bus.initialize()

        self.feedback_positions = [0] * 12

        for id in MOTOR_IDS.values():
            self.pcan_bus.set_motor_origin(motor_id=id)
            self.pcan_bus.enable_motor_mode(motor_id=id)

        if self.publish_joint_state:
            msg = JointState()

            msg.header.stamp = rospy.Time.now()
            msg.name = JOINT_NAMES
            msg.position = self.feedback_positions

            self.joint_state_publisher.publish(msg)

    def position_callback(self, msg):
        self.joint_positions = msg.points[0].positions

    def run_single_leg_loop(self, leg):

        while not rospy.is_shutdown():

            if not self.joint_positions:
                continue

            leg_joints = [name for name in JOINT_NAMES if name.startswith(leg)]
            leg_joint_positions = [self.joint_positions[self.joint_names.index(joint)] for joint in leg_joints]
            leg_motors = [motor for motor in self.motors.values() if motor.name.startswith(leg)]

            for motor, position in zip(leg_motors,leg_joint_positions):
                
                try:
                    feedback = self.pcan_bus.send_position(motor_id=motor.id, pos=motor.adjust_position(position))
                    print(feedback)

                except KeyboardInterrupt:
                    print("\nDisabling motor and exiting...")
                    
                    for id in MOTOR_IDS.values():
                        self.pcan_bus.disable_motor_mode(motor_id=id)
                    
                    self.pcan_bus.clean()
                    break

    def run_control_loop(self):
        
        while not rospy.is_shutdown():

            if not self.joint_positions:
                continue

            self.feedback_positions = []

            for motor, position in zip(self.motors.values(),self.joint_positions):
                
                try:
                    position_feedback = self.pcan_bus.send_position(motor_id=motor.id, pos=motor.adjust_position(position))
                    self.feedback_positions.append(motor.readjust_position(position_feedback))

                except KeyboardInterrupt:
                    print("\nDisabling motor and exiting...")
                    
                    for id in MOTOR_IDS.values():
                        self.pcan_bus.disable_motor_mode(motor_id=id)
                    
                    self.pcan_bus.clean()
                    break

            if self.publish_joint_state:
                msg = JointState()

                msg.header.stamp = rospy.Time.now()
                msg.name = JOINT_NAMES
                msg.position = self.feedback_positions

                self.joint_state_publisher.publish(msg)
            
              
if __name__ == "__main__":
    quadruped_controller = QuadrupedController()
    # quadruped_controller.run_control_loop()