# Robot Joint Names 
JOINT_NAMES = [
    'RF_HAA', 'RF_HFE', 'RF_KFE',   # Right Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
    'LF_HAA', 'LF_HFE', 'LF_KFE',   # Left Front
    'RH_HAA', 'RH_HFE', 'RH_KFE',   # Right Hind
    'LH_HAA', 'LH_HFE', 'LH_KFE'    # Left Hind
]


# Motor IDs 
MOTOR_IDS = {
    'RF_HAA':0x04, 'RF_HFE':0x05, 'RF_KFE':0x06,   # Right Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
    'LF_HAA':0x01, 'LF_HFE':0x02, 'LF_KFE':0x03,   # Left Front
    'RH_HAA':0x0A, 'RH_HFE':0x0B, 'RH_KFE':0x0C,   # Right Hind
    'LH_HAA':0x07, 'LH_HFE':0x08, 'LH_KFE':0x09    # Left Hind
             }


#lower leg limit to -55
# Min, Max and offset values in degrees 
# These offsets are for robot hanging on test bench with legs close
# MOTOR_MIN_MAX_OFFSET_MULT  = {
#     'RF_HAA':(-35, 35, 0, -1), 'RF_HFE':(-20, 90, 90, 1), 'RF_KFE':(-153, -55, -153, -0.88),   # Right Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
#     'LF_HAA':(-35, 35, 0, -1), 'LF_HFE':(-20, 90, 90, -1), 'LF_KFE':(-153, -55, -153, 0.88),   # Left Front
#     'RH_HAA':(-35, 35, 0, 1), 'RH_HFE':(-20, 90, 90, 1), 'RH_KFE':(-153, -55, -153, -0.88),   # Right Hind
#     'LH_HAA':(-35, 35, 0, 1), 'LH_HFE':(-20, 90, 90, -1), 'LH_KFE':(-153, -55, -153, 0.88)    # Left Hind
#              }

# These offesets are for robot lying on the ground with legs close
MOTOR_MIN_MAX_OFFSET_MULT  = {
    'RF_HAA':(-25, 15, -23, -1), 'RF_HFE':(-20, 90, 67, 1), 'RF_KFE':(-153, -55, -153, -0.88),   # Right Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
    'LF_HAA':(-15, 25, 23, -1), 'LF_HFE':(-20, 90, 67, -1), 'LF_KFE':(-153, -55, -153, 0.88),   # Left Front
    'RH_HAA':(-25, 15, -23, 1), 'RH_HFE':(-20, 90, 67, 1), 'RH_KFE':(-153, -55, -153, -0.88),   # Right Hind
    'LH_HAA':(-15, 25, 23, 1), 'LH_HFE':(-20, 90, 67, -1), 'LH_KFE':(-153, -55, -153, 0.88)    # Left Hind
             }


# Value limits
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -8.0, 8.0
KP_MIN, KP_MAX = 1.0, 100.0
KD_MIN, KD_MAX = 0.1, 5.0
T_MIN, T_MAX = -48.0, 48.0

# Default control values
V_IN, KP_IN, KD_IN, T_IN = 0.0, 0, 3.0, 1.0

