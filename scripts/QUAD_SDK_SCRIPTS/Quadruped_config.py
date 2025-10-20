# Robot Joint Names 
# JOINT_NAMES = [
#     'RF_HAA', 'RF_HFE', 'RF_KFE',   # Right Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
#     'LF_HAA', 'LF_HFE', 'LF_KFE',   # Left Front
#     'RH_HAA', 'RH_HFE', 'RH_KFE',   # Right Hind
#     'LH_HAA', 'LH_HFE', 'LH_KFE'    # Left Hind
# ]

# JOINT_NAMES = [
#     'LF_HAA',8 'LF_HFE',0 'LF_KFE',1   # Left Front
#     'LH_HAA',9 'LH_HFE',2 'LH_KFE' 3,    # Left Hind
#     'RF_HAA',10 'RF_HFE',4 'RF_KFE',5   # Right Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
#     'RH_HAA',11 'RH_HFE',6 'RH_KFE',7   # Right Hind

# ]

# Robot Joint Names 
# JOINT_NAMES = [
#     '8', '0', '1',   # Left Front
#     '9', '2', '3',    # Left Hind
#     '10', '4', '5',   # Right Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
#     '11', '6', '7',   # Right Hind

# ]
# JOINT_NAMES = [
#     '0', '1', '10',   # Left Front
#     '11', '2', '3',    # Left Hind
#     '4', '5', '6',   # Right Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
#     '7', '8', '9',   # Right Hind

# ]

JOINT_NAMES = [
    '8', '0', '1',   # Left Front
    '9', '2', '3',    # Left Hind
    '10', '4', '5',   # Right Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
    '11', '6', '7'   # Right Hind
]

# Motor IDs for INWARD BEND CONFIGURATION
# MOTOR_IDS = {
#     'LF_HAA':0x01, 'LF_HFE':0x02, 'LF_KFE':0x03,   # Left Front
#     'LH_HAA':0x07, 'LH_HFE':0x0B, 'LH_KFE':0x0C,    # Left Hind
#     'RF_HAA':0x04, 'RF_HFE':0x05, 'RF_KFE':0x06,   # Right Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
#     'RH_HAA':0x0A, 'RH_HFE':0x08, 'RH_KFE':0x09,   # Right Hind

#              }
# Motor IDs 
# MOTOR_IDS = {
#     '8':0x01, '0':0x02, '1':0x03,   # Left Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
#     '9':0x07, '2':0x08, '3':0x09,    # Left Hind
#     '10':0x04, '4':0x05, '5':0x06,   # Right Front 
#     '11':0x0A, '6':0x0B, '7':0x0C,   # Right Hind

#              }

# MOTOR_IDS = {
#     '0':0x02, '1':0x03, '10':0x04,   # Left Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
#     '11':0x0A,'2':0x08, '3':0x09,    # Left Hind
#     '4':0x05, '5':0x06, '6':0x0B, # Right Front 
#     '8':0x01,'9':0x07, '7':0x0C,   # Right Hind

#     }

MOTOR_IDS = {
    '8':0x01,  '0':0x02, '1':0x03,    # Left Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
    '9':0x07,  '2':0x08, '3':0x09,    # Left Hind
    '10':0x04, '4':0x05, '5':0x06,  # Right Front 
    '11':0x0A, '6':0x0B, '7':0x0C   # Right Hind
    }
# MOTOR_INDEX = {
#     'RF_HAA':0, 'RF_HFE':1, 'RF_KFE':2,   # Right Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
#     'LF_HAA':3, 'LF_HFE':4, 'LF_KFE':5,   # Left Front
#     'RH_HAA':6, 'RH_HFE':7, 'RH_KFE':8,   # Right Hind
#     'LH_HAA':9, 'LH_HFE':10, 'LH_KFE':11    # Left Hind
#              }

# These offesets are for QUAD_SDK
MOTOR_MIN_MAX_OFFSET_MULT  = {
    '8':(-25, 25,  -7, -1), '0':(-90, 110, -64,  1), '1':(28, 131, 28, 0.88),   # Left Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
    '9':(-25, 25,  -7,  1), '2':(-90, 110, -64,  1), '3':(28, 131, 28, 0.88),    # Left Hind
    '10':(-25, 25,  7, -1), '4':(-90, 110, -64, -1), '5':(28, 131, 28, -0.88),   # Right Front
    '11':(-25, 25,  7,  1), '6':(-90, 110, -64, -1), '7':(28, 131, 28, -0.88)   # Right Hind
             }

# MOTOR_MIN_MAX_OFFSET_MULT  = {
#     'LF_HAA':(-25, 25, -7, -1), 'LF_HFE':(-90, 110, -64,  1), 'LF_KFE':(48, 132, 48, 0.88),   # Left Front: Hip Abduction/Adduction, Hip Flexion/Extension, Knee Flexion/Extension
#     'LH_HAA':(-25, 25, -7,  1), 'LH_HFE':(-90, 110, -64,  1), 'LH_KFE':(48, 132, 48, 0.88),    # Left Hind
#     'RF_HAA':(-25, 25,  7, -1), 'RF_HFE':(-90, 110, -64, -1), 'RF_KFE':(48, 132, 48, -0.88),   # Right Front
#     'RH_HAA':(-25, 25,  7,  1), 'RH_HFE':(-90, 110, -64, -1), 'RH_KFE':(48, 132, 48, -0.88)   # Right Hind
#              }


# Value limits
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -5.0, 5.0
KP_MIN, KP_MAX = 1.0, 100.0
KD_MIN, KD_MAX = 0.1, 5.0
T_MIN, T_MAX = -144.0, 144.0

# Default control values
V_IN, KP_IN, KD_IN, T_IN = 0.0, 0, 2.0, 0.0

