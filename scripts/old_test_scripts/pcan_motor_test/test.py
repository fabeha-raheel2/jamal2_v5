MOTOR_IDS = [0x0A, 0x02, 0x03]

for motor_id in MOTOR_IDS:
    if motor_id == 0x2:
        continue
    print(hex(motor_id))