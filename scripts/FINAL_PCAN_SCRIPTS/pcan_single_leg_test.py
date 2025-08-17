from QuadrupedController import QuadrupedController

# No need to comment anything in the config file anymore.

leg = "lf"   # choose from "lf" (Left Front), "rf" (Right Front), "lh" (Left Hind), "rh" (Right Hind)

quadruped_controller = QuadrupedController()
quadruped_controller.run_single_leg_loop(leg=leg)