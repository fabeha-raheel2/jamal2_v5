from PcanController import *


# Parameters
Kp = 30
Kd = 3.0
motor_id = 0x00
    

if __name__ == "__main__":

    pcan_bus = PcanController()
    pcan_bus.initialize()

    last_position = 0.0



    while True:

        try:
            desired_position = float(input("Enter desired position: "))
            torque_cmd = Kp * (desired_position - last_position)
            feedback = pcan_bus.send_motor_data(motor_id=motor_id, pos=0, v_in=0, kp_in=0, kd_in=Kd, t_in=torque_cmd)
            last_position = feedback['position']
        except KeyboardInterrupt:
            print("\nDisabling motor and exiting...")
            pcan_bus.disable_motor_mode(motor_id=motor_id)                  
            pcan_bus.clean()
            break


