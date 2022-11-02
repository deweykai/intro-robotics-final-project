"""Robot manipulation

This robot controls all aspects the movement of the robot

This module depends on:
    * localization
    * navigation
"""

import robot
from robot import keyboard


class WheelMotors:
    def __init__(self):
        self.vL = 0
        self.vR = 0

    def update(self):
        robot.robot_parts["wheel_left_joint"].setVelocity(self.vL)
        robot.robot_parts["wheel_right_joint"].setVelocity(self.vR)


class Grippers:
    def __init__(self):
        self.left_finger = robot.robot_parts["gripper_left_finger_joint"]
        self.right_finger = robot.robot_parts["gripper_right_finger_joint"]


class ManualController:
    """Keyboard control robot controller"""

    def update(self):
        key = keyboard.getKey()
        if key == keyboard.LEFT:
            wheels.vL = -robot.MAX_SPEED
            wheels.vR = robot.MAX_SPEED
        elif key == keyboard.RIGHT:
            wheels.vL = robot.MAX_SPEED
            wheels.vR = -robot.MAX_SPEED
        elif key == keyboard.UP:
            wheels.vL = robot.MAX_SPEED
            wheels.vR = robot.MAX_SPEED
        elif key == keyboard.DOWN:
            wheels.vL = -robot.MAX_SPEED
            wheels.vR = -robot.MAX_SPEED
        elif key == ord(' '):
            wheels.vL = 0
            wheels.vR = 0
        else:  # slow down
            wheels.vL *= 0.75
            wheels.vR *= 0.75


gripper_status = "closed"

wheels = WheelMotors()
grippers = Grippers()
controller = ManualController()


def init():
    """Initialize manipulation module"""

    pass


def update():
    """Update hook for manipulation module"""

    global gripper_status
    controller.update()
    wheels.update()

    if gripper_status == "open":
        # Close gripper, note that this takes multiple time steps...
        grippers.left_finger.setPosition(0)
        grippers.right_finger.setPosition(0)
        if robot.right_gripper_enc.getValue() <= 0.005:
            gripper_status = "closed"
    else:
        # Open gripper
        grippers.left_finger.setPosition(0.045)
        grippers.right_finger.setPosition(0.045)

        if robot.left_gripper_enc.getValue() >= 0.044:
            gripper_status = "open"
