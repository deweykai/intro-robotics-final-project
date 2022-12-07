import robot
import bus


def left_wheel_cmd(value):
    robot.robot_parts["wheel_left_joint"].setVelocity(value)


bus.Subscriber('/bot/wheel/cmd_vel/left', float, left_wheel_cmd)


def right_wheel_cmd(value):
    robot.robot_parts["wheel_right_joint"].setVelocity(value)


bus.Subscriber('/bot/wheel/cmd_vel/right', float, right_wheel_cmd)

bus.publish_once('/bot/wheel/cmd_vel/left', float, 0.0)
bus.publish_once('/bot/wheel/cmd_vel/right', float, 0.0)
