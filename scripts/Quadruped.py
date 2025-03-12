#!/usr/bin/env python

from math import pi, acos, asin, sqrt, cos, sin, degrees
import rospy
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time


class QuadrupedJointControllers:
    def __init__(self, ns="jamal2"):
        # rospy.init_node('quadruped_joint_publisher', anonymous=True)

        self.ns = ns

        self.joint_names = [
        "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",  
        "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint", 
        "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
        "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        ]

        self.leg_joints = {'rf': ["rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint"],
                           'lf': ["lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint"],
                           'rh': ["rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"],
                           'lh': ["lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"]
                           }


        self.joint_publishers = {joint:rospy.Publisher(f'/{self.ns}/{joint}_position_controller/command', Float64, queue_size=10) for joint in self.joint_names}

        self.leg_publishers = {'rf': [self.joint_publishers['rf_hip_joint'], self.joint_publishers['rf_upper_leg_joint'], self.joint_publishers['rf_lower_leg_joint']],
                               'lf': [self.joint_publishers['lf_hip_joint'], self.joint_publishers['lf_upper_leg_joint'], self.joint_publishers['lf_lower_leg_joint']],
                               'rh': [self.joint_publishers['rh_hip_joint'], self.joint_publishers['rh_upper_leg_joint'], self.joint_publishers['rh_lower_leg_joint']],
                               'lh': [self.joint_publishers['lh_hip_joint'], self.joint_publishers['lh_upper_leg_joint'], self.joint_publishers['lh_lower_leg_joint']]}

        self.target_positions = {}

        self.sit_target_positions = {
        "lh_hip_joint": 0.0, "lh_upper_leg_joint": 1.5, "lh_lower_leg_joint": -2.5,
        "lf_hip_joint": 0.0, "lf_upper_leg_joint": 1.5, "lf_lower_leg_joint": -2.5,
        "rh_hip_joint": 0.0, "rh_upper_leg_joint": 1.5, "rh_lower_leg_joint": -2.5,
        "rf_hip_joint": 0.0, "rf_upper_leg_joint": 1.5, "rf_lower_leg_joint": -2.5
        }

        self.stand_target_positions = {
        "lh_hip_joint": 0.0, "lh_upper_leg_joint": 0.8, "lh_lower_leg_joint": -1.5,
        "lf_hip_joint": 0.0, "lf_upper_leg_joint": 0.8, "lf_lower_leg_joint": -1.5,
        "rh_hip_joint": 0.0, "rh_upper_leg_joint": 0.8, "rh_lower_leg_joint": -1.5,
        "rf_hip_joint": 0.0, "rf_upper_leg_joint": 0.8, "rf_lower_leg_joint": -1.5
        }
        
        self.current_positions = {joint: 0.0 for joint in self.sit_target_positions}  # Default values
        
        rospy.Subscriber('/jamal2/joint_states', JointState, self.joint_state_callback)
        rospy.loginfo("Waiting for joint states...")
        rospy.sleep(2)  # Give time to receive initial joint states
    
    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]
                # print(self.current_positions)
    
class Quadruped:
    def __init__(self, link_lengths=[1, 1, 1], bend="backward", ns="jamal2", debug_mode=False, test_mode=False):
        # LEG PARAMS
        self.link_lengths = link_lengths    
        self.FORWARD_BEND  = bend != "backward"    # True if legs are in Forward Bend condition, else False
        self.ns = ns

        self.DEBUG = debug_mode
        self.TEST = test_mode

        self.a = 1.0
        self.T = 3.0
        self.compute_velocity()

        self.epsilon = 1e-6

        if not self.TEST:
            rospy.init_node('jamal2_node', anonymous=True)

            self.controllers = QuadrupedJointControllers(ns=self.ns)

    def set_link_lengths(self, link_lengths):
        self.link_lengths = link_lengths

    def get_link_lengths(self):
        return self.link_lengths


    def compute_inv_kinematics(self, position=[-1 , -1 , -1]):

        x, y, z = position
        assert y < 0, "Invalid y coordinate input!"
        assert y**2 + z**2 >= self.link_lengths[0]**2, "Invalid leg position: y^2 + z^2 must be >= l1^2."

        alpha = acos(abs(z) / sqrt(y**2 + z**2))
        beta = acos(self.link_lengths[0] / sqrt(y**2 + z**2))

        if z>=0: q1 = alpha - beta  
        else: q1 = pi - alpha - beta

        y_prime = -1*sqrt(y**2 + z**2 - self.link_lengths[0]**2)

        phi = acos(abs(x) / sqrt(x**2 + y_prime**2))
        psi = acos((self.link_lengths[1]**2 + x**2 + y_prime**2 - self.link_lengths[2]**2) / (2*self.link_lengths[1]*sqrt(x**2 + y_prime**2)))
        
        if not self.FORWARD_BEND:
            if x>0: q2 = (pi/2 - psi - phi) 
            else: q2 = (-pi/2 - psi + phi)
            q3 = pi - acos((self.link_lengths[1]**2 + self.link_lengths[2]**2 - x**2 - y_prime**2) / (2*self.link_lengths[1]*self.link_lengths[2]))
        else:
            if x>0: q2 = (pi/2 + psi - phi)
            else: q2 = (-pi/2 + psi + phi)
            q3 = -1*(pi - acos((self.link_lengths[1]**2 + self.link_lengths[2]**2 - x**2 - y_prime**2) / (2*self.link_lengths[1]*self.link_lengths[2])))

        assert abs(q2) < pi/2, "q2 greater than limits"

        if self.DEBUG:
            print("alpha: ", alpha)
            print("beta: ", beta)
            print("yprime: ", y_prime)
            print("phi: ", phi)
            print("psi: ", psi)
            print("q1: ", degrees(q1))
            print("q2: ", degrees(q2))
            print("q3: ", degrees(q3))

        return (q1, q2, q3)
    
    
    def publish_joint_commands(self, commands):
        
        for pub, command in zip(self.controllers.joint_publishers.items(), commands):
            try:
                pub.publish(Float64(command))
            except rospy.ROSException as e:
                rospy.logerr(f"Failed to publish command: {e}")

    def publish_leg_commands(self, leg, joint_angles):
        
        for pub, angle in zip(self.controllers.leg_publishers[leg],joint_angles):
            try:
                pub.publish(Float64(angle))
            except rospy.ROSException as e:
                rospy.logerr(f"Failed to publish command: {e}")
    
    def sit(self):
        rospy.loginfo("Gradually moving to sit in all-elbow configuration...")
        rate = rospy.Rate(10) 
        self.controllers.target_positions = self.controllers.sit_target_positions
        
        rospy.loginfo("Moving to sit position...")
        for i in range(20):  # Smooth transition over 2 seconds
            for joint, pub in self.controllers.joint_publishers.items():
                new_value = self.controllers.stand_target_positions[joint] + (self.controllers.sit_target_positions[joint] - self.controllers.stand_target_positions[joint]) * (i / 20.0)
                pub.publish(Float64(new_value))
            rate.sleep()
        rospy.sleep(2)  # Hold sitting position for 2 seconds


    def stand(self):
        rospy.loginfo("Gradually moving to stand in all-elbow configuration...")
        rate = rospy.Rate(10)  
        self.controllers.target_positions = self.controllers.stand_target_positions
        
        rospy.loginfo("Moving to stand position...")
        for i in range(20):  # Smooth transition over 2 seconds
            for joint, pub in self.controllers.joint_publishers.items():
                new_value = self.controllers.sit_target_positions[joint] + (self.controllers.stand_target_positions[joint] - self.controllers.sit_target_positions[joint]) * (i / 20.0)
                pub.publish(Float64(new_value))
            rate.sleep()
        rospy.sleep(2)  # Hold sitting position for 2 seconds

    def circular_trajectory(self, initial_position=[0, 0, 0], radius=0.1, n_points=20):

        xc, yc = initial_position[0]+radius, initial_position[1]
        z = initial_position[2]
        trajectory = []
        angles = np.linspace(0, np.pi, n_points)

        for angle in angles:
            trajectory.append([xc + radius*np.cos(angle), yc + radius*np.sin(angle), z])

        return trajectory
    
    def compute_velocity(self):
        self.t_a = (self.a * self.T) / (2 + self.a)
        self.v = self.a * self.t_a
    
    def elliptical_trajectory(self, x_pre, y_pre, z_pre, L, H, W, s_func, t_range):

        x = x_pre + L + L * np.cos(np.pi - s_func(t_range))
        y = y_pre + H * np.sin(np.pi - s_func(t_range))
        z = z_pre + W + W * np.cos(np.pi - s_func(t_range))

        return x, y, z
    
    def linear_displacement(self, t):
        return t

    def trapezoidal_profile(self, t):
        t_a_mask1 = t <= self.t_a
        t_a_mask2 = (t > self.t_a) & (t <= self.T - self.t_a)

        s = np.where(
            t_a_mask1,
            0.5 * self.a * t**2,
            np.where(
                t_a_mask2,
                self.v * t - (self.v**2) / (2 * self.a),
                (2 * self.a * self.v * self.T - 2 * self.v**2 - self.a**2 * (t - self.T)**2) / (2 * self.a)
            )
        )

        s_max = s[-1]
        normalized_s = (s / s_max) * np.pi
        return normalized_s
    
    def translation(self, x_start, x_end, y_start, y_end, z_start, z_end, t_range):
        x = np.linspace(x_start, x_end, len(t_range))
        y = np.linspace(y_start, y_end, len(t_range))
        z = np.linspace(z_start, z_end, len(t_range))

        return x, y, z
    
    def move_single_leg_ellipse(self, leg, x_pre, y_pre, z_pre, L, H, W, t_range):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            x_e, y_e, z_e = self.elliptical_trajectory(x_pre, y_pre, z_pre, L, H, W, self.trapezoidal_profile, t_range)
            rospy.loginfo("Calculated elliptical traj")

            for i in range(len(x_e)):
                curr_x_e = x_e[i]
                curr_y_e = y_e[i]
                curr_z_e = z_e[i]

                rospy.loginfo("Current trajectory values: x_e={}, y_e={}, z_e={}".format(curr_x_e, curr_y_e, curr_z_e))

                joint_angles = self.compute_inv_kinematics([curr_x_e, curr_y_e, curr_z_e])

                rospy.loginfo("Publishing joint commands: {}".format(joint_angles))
                self.publish_leg_commands(leg=leg, joint_angles=joint_angles)
                    
                rate.sleep()

            x_t_out, y_t, z_t = self.translation(L*2, x_pre, y_pre, y_pre, z_pre, z_pre, t_range) # -0.1, 0.0

            for i in range(len(x_e)):
                curr_x_t_out = x_t_out[i]
                curr_y_t = y_t[i]
                curr_z_t = z_t[i]

                rospy.loginfo("Current trajectory values: x_t_out={}, x_e_out={}, y_t={}, z_t={}".format(curr_x_t_out, curr_x_t_out, curr_y_t, curr_z_t))

                joint_angles = self.compute_inv_kinematics([curr_x_t_out, curr_y_t, curr_z_t])

                rospy.loginfo("Publishing joint commands: {}".format(joint_angles))
                self.publish_leg_commands(leg=leg, joint_angles=joint_angles)
                    
                rate.sleep()
    
    def move_leg_circular(self, leg='lf', inital_position=[0, -0.29, 0.105], radius=0.1, n_points=20):

        trajectory = self.circular_trajectory(initial_position=inital_position, radius=radius, n_points=n_points)

        for point in trajectory:
            angles = self.compute_inv_kinematics(position=point)
            self.publish_leg_commands(leg=leg, joint_angles=angles)
            time.sleep(1)

    

if __name__ == "__main__":

    # INPUTS
    position = [-0.05 , -0.324285661 , 0.17]
    L1 = 0.105  # Length of Link 1 (Hip) m
    L2 = 0.225  # Length of Link 2 (Thigh) m
    L3 = 0.230  # Length of Link 3 (Lower Leg / Shin) m

    robot = Quadruped(debug_mode=True)
    robot.set_link_lengths([L1, L2, L3])

    
    #### TEST SINGLE LEG IK ...................................................
    
    # joint_angles = robot.compute_inv_kinematics(position)

    # print("Calculated Joint Angles:")
    # print("Theta 1: ", math.degrees(joint_angles[0]))
    # print("Theta 2: ", math.degrees(joint_angles[1]))
    # print("Theta 3: ", math.degrees(joint_angles[2]))


    ### TEST QUADRUPED SIT .......................................................
    robot.sit()
    robot.stand()
