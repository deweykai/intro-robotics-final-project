import robot

vL = 0
vR = 0

gripper_status = "closed"


def init():
    pass


def update():
    global gripper_status
    robot.robot_parts["wheel_left_joint"].setVelocity(vL)
    robot.robot_parts["wheel_right_joint"].setVelocity(vR)

    if gripper_status == "open":
        # Close gripper, note that this takes multiple time steps...
        robot.robot_parts["gripper_left_finger_joint"].setPosition(0)
        robot.robot_parts["gripper_right_finger_joint"].setPosition(0)
        if robot.right_gripper_enc.getValue() <= 0.005:
            gripper_status = "closed"
    else:
        # Open gripper
        robot.robot_parts["gripper_left_finger_joint"].setPosition(0.045)
        robot.robot_parts["gripper_right_finger_joint"].setPosition(0.045)
        if robot.left_gripper_enc.getValue() >= 0.044:
            gripper_status = "open"
