from QuadrupedController import QuadrupedController

# Remember to make appropriate changes in config file. 

# Comment all non-necessary joints in MOTOR_IDS and MOTOR_MIN_MAX_OFFSET_MULT

leg = "lf"   # choose from "lf" (Left Front), "rf" (Right Front), "lh" (Left Hind), "rh" (Right Hind)

quadruped_controller = QuadrupedController()
quadruped_controller.run_single_leg_loop(leg=leg)