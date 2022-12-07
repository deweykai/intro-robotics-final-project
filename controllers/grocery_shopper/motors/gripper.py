import robot
import bus

gripper_parts = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
for part in gripper_parts:
    max_vel = robot.robot_parts[part].getMaxVelocity()
    robot.robot_parts[part].setVelocity(max_vel)

@bus.subscribe('/bot/cmd_gripper', bool)
def cmd_gripper(open_state):
    if open_state:
        robot.robot_parts["gripper_left_finger_joint"].setPosition(0.045)
        robot.robot_parts["gripper_right_finger_joint"].setPosition(0.045)
    else:
        robot.robot_parts["gripper_left_finger_joint"].setPosition(0)
        robot.robot_parts["gripper_right_finger_joint"].setPosition(0)
