from Quadruped_config import *
import random

leg = "rf"

joint_names = JOINT_NAMES

joint_positions = [random.random() for _ in range(12)]
# print(joint_positions)

leg_joints = [name for name in JOINT_NAMES if name.startswith(leg)]
# print(leg_joints)

leg_joint_positions = [joint_positions[joint_names.index(joint)] for joint in leg_joints]

print("Joint positions: ", joint_positions)
print("leg positions: ", leg_joint_positions)

# for motor, position in zip(motors.values(),self.joint_positions):
#     pass