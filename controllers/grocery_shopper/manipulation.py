"""Robot manipulation

This robot controls all aspects the movement of the robot

This module depends on:
    * localization
    * navigation
"""

import robot
from robot import keyboard
import mapping
import navigation
import localization as loc
import numpy as np

DEBUG = True

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

    def __init__(self) -> None:
        print('Starting manual controls')

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
        elif key == ord('S'):
            mapping.mapper.save()
        elif key == ord('L'):
            mapping.mapper.load()
        elif key == ord('A'):
            global controller
            controller = IKController(navigation.waypoints)
        else:  # slow down
            wheels.vL *= 0.75
            wheels.vR *= 0.75


class IKController:
    """Inverse Kinematics Controller"""

    def __init__(self, waypoints) -> None:
        self.waypoints = waypoints
        self.checkpoint = 0

    def target_reached(self):
        return self.checkpoint >= len(self.waypoints)

    def target_pos(self):
        if self.target_reached():
            # reached target position
            return loc.pose_x, loc.pose_y
        else:
            return self.waypoints[self.checkpoint]

    def calculate_position_error(self):
        target_x, target_y = self.target_pos()
        return np.sqrt((loc.pose_x - target_x)**2 + (loc.pose_y - target_y)**2)


    def calculate_bearing_error(self):
        target_x, target_y = self.target_pos()
        dx, dy = target_x - loc.pose_x, target_y - loc.pose_y
        theta = np.arctan2(dy, dx)
        return theta - loc.pose_theta

    def update(self):
        if self.target_reached():
            global controller
            controller = ManualController()
            return 

        def inverse_kinematics(dx, theta):
            L = dx - theta * robot.AXLE_LENGTH / 2
            R = dx + theta * robot.AXLE_LENGTH / 2
            return L, R

        def balance_values(vL, vR):
            '''normalize vL, vR to -1 to 1'''
            max_value = max(abs(vL), abs(vR))
            max_value = max(max_value, robot.MAX_SPEED)
            vR = vR / max_value
            vL = vL / max_value
            return vL, vR

        rho = self.calculate_position_error()
        alpha = self.calculate_bearing_error()

        # check if it has hit the waypoint
        if rho < 0.5:
            print(f'hit waypoint {self.checkpoint}')
            self.checkpoint += 10
            return

        # STEP 2: Controller
        dx = 3 * rho
        dtheta = 10 * alpha

        # STEP 3: Compute wheelspeeds
        vL, vR = inverse_kinematics(dx, dtheta)
        vL, vR = balance_values(vL, vR)  # normalize values between -1 to 1

        vL = vL * robot.MAX_SPEED  # convert to rotational vel
        vR = vR * robot.MAX_SPEED  # convert to rotational vel

        if DEBUG:
            print('==== NAVIGATION FRAME ===')
            print(f'[pose_x = {loc.pose_x:.3}, pose_y = {loc.pose_y:.3}, pose_theta = {loc.pose_theta:.3}]')
            print(f'target = {self.target_pos()}')
            print(f'rho = {rho}')
            print(f'alpha = {alpha}')
            print(f'vL = {vL}')
            print(f'vR = {vL}')

        # Normalize wheelspeed
        # (Keep the wheel speeds a bit less than the actual platform MAX_SPEED to minimize jerk)
        wheels.vL, wheels.vR = vL, vR

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
