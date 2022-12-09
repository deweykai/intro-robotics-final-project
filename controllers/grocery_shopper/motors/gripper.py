"""Gripper

Open and close the robot gripper.
"""

import robot
import bus

close_position = 0.0
open_position = 0.045

gripper_parts = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
for part in gripper_parts:
    max_vel = robot.robot_parts[part].getMaxVelocity()
    robot.robot_parts[part].setVelocity(max_vel)


@bus.subscribe('/bot/cmd_gripper', bool)
def cmd_gripper(open_state):
    if open_state:
        robot.robot_parts["gripper_left_finger_joint"].setPosition(
            open_position)
        robot.robot_parts["gripper_right_finger_joint"].setPosition(
            open_position)
    else:
        robot.robot_parts["gripper_left_finger_joint"].setPosition(
            close_position)
        robot.robot_parts["gripper_right_finger_joint"].setPosition(
            close_position)
