from scripts.pcan_motor_test.PCANBasic import *
from ctypes import c_ubyte
import time

pcan = PCANBasic()
channel = PCAN_USBBUS1  # Linux still uses this constant
baudrate = PCAN_BAUD_1M

if pcan.Initialize(channel, baudrate) != PCAN_ERROR_OK:
    print("Failed to initialize PCAN.")
    exit(1)

msg = TPCANMsg()
msg.ID = 0x123
msg.LEN = 8
msg.MSGTYPE = PCAN_MESSAGE_STANDARD
msg.DATA = (c_ubyte * 8)(0xDE, 0xAD, 0xBE, 0xEF, 0x11, 0x22, 0x33, 0x44)

result = pcan.Write(channel, msg)
if result == PCAN_ERROR_OK:
    print("Message sent successfully.")
else:
    print("Failed to send message. Error:", pcan.GetErrorText(result)[1])

pcan.Uninitialize(channel)
