#!/usr/bin/env python3

from PCANBasic import *
from ctypes import c_ubyte
import time

from Quadruped_config import *

class PcanController:
    def __init__(self):
        self.pcan = PCANBasic()
        self.channel = PCAN_USBBUS1
        self.baudrate = PCAN_BAUD_1M
        self.frequency = 50     #hertz
        self.delay = 1/self.frequency

        # Motor mode commands
        self.MotorModeOn = [0xFF]*7 + [0xFC]
        self.MotorModeOff = [0xFF]*7 + [0xFD]
        self.SetOrigin = [0xFF]*7 + [0xFE]

    def initialize(self):
        self.pcan.Initialize(self.channel, self.baudrate)
        self.pcan.SetValue(self.channel, PCAN_RECEIVE_EVENT, 0)

        self.v_in, self.kp_in, self.kd_in, self.t_in = V_IN, KP_IN, KD_IN, T_IN
        
        time.sleep(self.delay)

    def set_motor_origin(self, motor_id):
        while True:
            self.send_can_msg(self.SetOrigin, id=motor_id)
            time.sleep(self.delay)
            result, _, _ = self.pcan.Read(self.channel)
            if result != PCAN_ERROR_QRCVEMPTY:
                break
    
    def enable_motor_mode(self, motor_id):
        while True:
            self.send_can_msg(self.MotorModeOn, id=motor_id)
            time.sleep(0.1)
            result, _, _ = self.pcan.Read(self.channel)
            if result != PCAN_ERROR_QRCVEMPTY:
                break

    def disable_motor_mode(self, motor_id):
        while True:
            self.send_can_msg(self.MotorModeOff, id=motor_id)
            time.sleep(0.1)
            result, _, _ = self.pcan.Read(self.channel)
            if result != PCAN_ERROR_QRCVEMPTY:
                break

    def send_position(self, motor_id, pos):
        data = self.pack_cmd(pos, self.v_in, self.kp_in, self.kd_in, self.t_in)
        self.send_can_msg(data, id=motor_id)
        return self.receive_can_msg()['position']
    
    def send_motor_data(self, motor_id, pos, v_in, kp_in, kd_in, t_in):
        data = self.pack_cmd(pos, v_in, kp_in, kd_in, t_in)
        self.send_can_msg(data, id=motor_id)
        return self.receive_can_msg()
    
    def clean(self):
        # self.disable_motor_mode()
        self.pcan.Uninitialize(self.channel)  

    def send_can_msg(self, data, id):
        msg = TPCANMsg() 
        msg.ID = id
        msg.LEN = 8
        msg.MSGTYPE = PCAN_MESSAGE_STANDARD
        msg.DATA = (c_ubyte * 8)(*data)
        self.pcan.Write(self.channel, msg)
        time.sleep(self.delay)

    def receive_can_msg(self):
        result, msg, timestamp = self.pcan.Read(self.channel)
        if result == PCAN_ERROR_OK:
            buf = list(msg.DATA)
            p_int = (buf[1] << 8) | buf[2]
            v_int = (buf[3] << 4) | (buf[4] >> 4)
            t_int = ((buf[4] & 0xF) << 8) | buf[5]
            p_out = self.uint_to_float(p_int, P_MIN, P_MAX, 16)
            v_out = self.uint_to_float(v_int, V_MIN, V_MAX, 12)
            t_out = self.uint_to_float(t_int, -T_MAX, T_MAX, 12)

            output = f"Measured -> Pos: {p_out:.2f}, Vel: {v_out:.2f}, Trq: {t_out:.2f}"
            # print(output)
        else:
            output = "No response from actuator."
            print(output)

        return {'position':p_out, 'velocity':v_out, 'torque': t_out}

    def pack_cmd(self, p_in, v_in, kp_in, kd_in, t_in):
        p_des = max(min(p_in, P_MAX), P_MIN)
        v_des = max(min(v_in, V_MAX), V_MIN)
        kp = max(min(kp_in, KP_MAX), KP_MIN)
        kd = max(min(kd_in, KD_MAX), KD_MIN)
        t_ff = max(min(t_in, T_MAX), T_MIN)

        p_int = self.float_to_uint(p_des, P_MIN, P_MAX, 16)
        v_int = self.float_to_uint(v_des, V_MIN, V_MAX, 12)
        kp_int = self.float_to_uint(kp, KP_MIN, KP_MAX, 12)
        kd_int = self.float_to_uint(kd, KD_MIN, KD_MAX, 12)
        t_int = self.float_to_uint(t_ff, T_MIN, T_MAX, 12)

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
    
    def float_to_uint(self, x, x_min, x_max, bits):
        span = x_max - x_min
        offset = x_min
        if bits == 12:
            return int((x - offset) * 4095.0 / span)
        elif bits == 16:
            return int((x - offset) * 65535.0 / span)
        return 0

    def uint_to_float(self, x_int, x_min, x_max, bits):
        span = x_max - x_min
        offset = x_min
        if bits == 12:
            return x_int * span / 4095.0 + offset
        elif bits == 16:
            return x_int * span / 65535.0 + offset
        return 0.0
