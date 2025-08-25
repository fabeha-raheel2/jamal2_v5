from QuadrupedController import QuadrupedController

# If you want to publish joint states then set publish_joint_state argument to True,
# Else, keep it False.

quadruped_controller = QuadrupedController(publish_joint_state=True,
                                           debug=True)
quadruped_controller.run_control_loop()