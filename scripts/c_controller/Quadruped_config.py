# Robot Joint Names 
JOINT_NAMES = ['lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint', 
               'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint', 
               'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint', 
               'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint'
               ]

##### These commented values are for backward bend >> configuration
# # Motor IDs 
# MOTOR_IDS = {'lf_hip_joint':0x01, 'lf_upper_leg_joint':0x02, 'lf_lower_leg_joint':0x03,
#              'rf_hip_joint':0x04, 'rf_upper_leg_joint':0x05, 'rf_lower_leg_joint':0x06,
#              'lh_hip_joint':0x07, 'lh_upper_leg_joint':0x08, 'lh_lower_leg_joint':0x09,
#              'rh_hip_joint':0x0A, 'rh_upper_leg_joint':0x0B, 'rh_lower_leg_joint':0x0C
#              }

# # Min, Max and offset values in degrees 
# MOTOR_MIN_MAX_OFFSET_MULT = {'lf_hip_joint':(-35, 35, 0, -1), 'lf_upper_leg_joint':(-20, 90, 90, -1), 'lf_lower_leg_joint':(-153, -47, -153, 0.88),
#                              'rf_hip_joint':(-35, 35, 0, -1), 'rf_upper_leg_joint':(-20, 90, 90, 1), 'rf_lower_leg_joint':(-153, -47, -153, -0.88),
#                              'lh_hip_joint':(-35, 35, 0, 1), 'lh_upper_leg_joint':(-20, 90, 90, -1), 'lh_lower_leg_joint':(-153, -47, -153, 0.88),
#                              'rh_hip_joint':(-35, 35, 0, 1), 'rh_upper_leg_joint':(-20, 90, 90, 1), 'rh_lower_leg_joint':(-153, -47, -153, -0.88)
#                              }

# Motor IDs 
MOTOR_IDS = {'lf_hip_joint':0x01, 'lf_upper_leg_joint':0x02, 'lf_lower_leg_joint':0x03,
             'rf_hip_joint':0x04, 'rf_upper_leg_joint':0x05, 'rf_lower_leg_joint':0x06,
             'lh_hip_joint':0x07, 'lh_upper_leg_joint':0x0B, 'lh_lower_leg_joint':0x0C,
             'rh_hip_joint':0x0A, 'rh_upper_leg_joint':0x08, 'rh_lower_leg_joint':0x09
             }

# Min, Max and offset values in degrees 
MOTOR_MIN_MAX_OFFSET_MULT = {'lf_hip_joint':(-25, 25, 0, -1), 'lf_upper_leg_joint':(-20, 90, 90, -1), 'lf_lower_leg_joint':(-153, -47, -153, 0.88),
                             'rf_hip_joint':(-25, 25, 0, -1), 'rf_upper_leg_joint':(-20, 90, 90, 1), 'rf_lower_leg_joint':(-153, -47, -153, -0.88),
                             'lh_hip_joint':(-25, 25, 0, 1), 'lh_upper_leg_joint':(-81, 20, -90, -1), 'lh_lower_leg_joint':(47, 153, 153, 0.88),
                             'rh_hip_joint':(-25, 25, 0, 1), 'rh_upper_leg_joint':(-90, 20, -90, 1), 'rh_lower_leg_joint':(47, 153, 153, -0.88)
                             }


# Value limits
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -8.0, 8.0
KP_MIN, KP_MAX = 1.0, 100.0
KD_MIN, KD_MAX = 0.1, 5.0
T_MIN, T_MAX = -144.0, 144.0

# Default control values
V_IN, KP_IN, KD_IN, T_IN = 0.0, 30.0, 3.0, 1.0

