from Quadruped_config import *
import random
import math

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
    
    def __str__(self):
        return f"Motor {self.name} with ID {self.id}"
    
    def __repr__(self):
        return f"Motor {self.name} with ID {self.id}"
    
motors = MOTOR_IDS.copy()

for motor in MOTOR_IDS.keys():
    motors[motor] = Motor(name=motor,
                                id=MOTOR_IDS[motor],
                                min_value=math.radians(MOTOR_MIN_MAX_OFFSET_MULT[motor][0]),
                                max_value=math.radians(MOTOR_MIN_MAX_OFFSET_MULT[motor][1]),
                                offset=math.radians(MOTOR_MIN_MAX_OFFSET_MULT[motor][2]),
                                multiplier=MOTOR_MIN_MAX_OFFSET_MULT[motor][3])

leg = "rf"

joint_names = JOINT_NAMES

joint_positions = [random.random() for _ in range(12)]
# print(joint_positions)

leg_joints = [name for name in JOINT_NAMES if name.startswith(leg)]
# print(leg_joints)

leg_joint_positions = [joint_positions[joint_names.index(joint)] for joint in leg_joints]

leg_motors = [motor for motor in motors.values() if motor.name.startswith(leg)]
# print("Joint positions: ", joint_positions)
# print("leg positions: ", leg_joint_positions)
# print(leg_motors)

# for motor, position in zip(motors.values(),leg_joint_positions):
#     print(f"Motor: {motor}, Position: {position}")

list = [0] * 12

print(list)