import robot
import bus


@bus.subscribe('/bot/cmd_gripper', bool)
def cmd_gripper(open_state):
    if open_state:
        robot.robot_parts["gripper_left_finger_joint"].setPosition(0.045)
        robot.robot_parts["gripper_right_finger_joint"].setPosition(0.045)
    else:
        robot.robot_parts["gripper_left_finger_joint"].setPosition(0)
        robot.robot_parts["gripper_right_finger_joint"].setPosition(0)
