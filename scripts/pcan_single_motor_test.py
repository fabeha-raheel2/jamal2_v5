from PCANBasic import *
from ctypes import c_ubyte
import time

# Initialize PCAN
pcan = PCANBasic()
channel = PCAN_USBBUS1
baudrate = PCAN_BAUD_1M
pcan.Initialize(channel, baudrate)
pcan.SetValue(channel, PCAN_RECEIVE_EVENT, 0)

# Motor mode commands
MotorModeOn = [0xFF]*7 + [0xFC]
MotorModeOff = [0xFF]*7 + [0xFD]

# Value limits
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -8.0, 8.0
KP_MIN, KP_MAX = 1.0, 100.0
KD_MIN, KD_MAX = 0.1, 5.0
T_MIN, T_MAX = -144.0, 144.0

# Default control values
v_in, kp_in, kd_in = 0.0, 200.0, 5.0

def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    if bits == 12:
        return int((x - offset) * 4095.0 / span)
    elif bits == 16:
        return int((x - offset) * 65535.0 / span)
    return 0

def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    if bits == 12:
        return x_int * span / 4095.0 + offset
    elif bits == 16:
        return x_int * span / 65535.0 + offset
    return 0.0

def send_can_msg(data):
    msg = TPCANMsg()
    msg.ID = 0x01
    msg.LEN = 8
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD
    msg.DATA = (c_ubyte * 8)(*data)
    pcan.Write(channel, msg)

def receive_can_msg():
    result, msg, timestamp = pcan.Read(channel)
    if result == PCAN_ERROR_OK:
        buf = list(msg.DATA)
        p_int = (buf[1] << 8) | buf[2]
        v_int = (buf[3] << 4) | (buf[4] >> 4)
        t_int = ((buf[4] & 0xF) << 8) | buf[5]
        p_out = uint_to_float(p_int, P_MIN, P_MAX, 16)
        v_out = uint_to_float(v_int, V_MIN, V_MAX, 12)
        t_out = uint_to_float(t_int, -T_MAX, T_MAX, 12)
        print(f"Measured -> Pos: {p_out:.2f}, Vel: {v_out:.2f}, Trq: {t_out:.2f}")
    else:
        print("No response from actuator.")

def pack_cmd(p_in, v_in, kp_in, kd_in, t_in):
    p_des = max(min(p_in, P_MAX), P_MIN)
    v_des = max(min(v_in, V_MAX), V_MIN)
    kp = max(min(kp_in, KP_MAX), KP_MIN)
    kd = max(min(kd_in, KD_MAX), KD_MIN)
    t_ff = max(min(t_in, T_MAX), T_MIN)

    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)

    buf = [
        (p_int >> 8) & 0xFF,
        p_int & 0xFF,
        (v_int >> 4) & 0xFF,
        ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF),
        kp_int & 0xFF,
        (kd_int >> 4) & 0xFF,
        ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF),
        t_int & 0xFF,
    ]
    return buf

# Enable motor mode
print("Enabling motor mode...")
while True:
    send_can_msg(MotorModeOn)
    time.sleep(0.1)
    result, _, _ = pcan.Read(channel)
    if result != PCAN_ERROR_QRCVEMPTY:
        break
print("Motor mode enabled.")

# User input loop
try:
    while True:
        try:
            p_in = float(input("Enter desired position (p_in): "))
            t_in = float(input("Enter desired torque (t_in): "))
        except ValueError:
            print("Invalid input. Try again.")
            continue

        can_data = pack_cmd(p_in, v_in, kp_in, kd_in, t_in)
        send_can_msg(can_data)
        time.sleep(0.01)
        receive_can_msg()

except KeyboardInterrupt:
    print("\nDisabling motor and exiting...")
    send_can_msg(MotorModeOff)
    pcan.Uninitialize(channel)
