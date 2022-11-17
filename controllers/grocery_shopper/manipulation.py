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
import task_tree
import logging

logger = logging.getLogger(__name__)
nav_logger = logger.getChild('navigation')
nav_logger.setLevel(logging.DEBUG)

autonomous = False

POSITION_THRESHOLD = 0.1


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
        self.auto_cooldown = 0
        logger.info('Starting manual controls')

    def update(self):
        self.auto_cooldown = max(self.auto_cooldown - 1, 0)
        global autonomous
        key = keyboard.getKey()
        if not autonomous:
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

        if key == ord('S'):
            mapping.mapper.save()
        elif key == ord('L'):
            mapping.mapper.load()
        elif key == ord('A'):
            if self.auto_cooldown > 0:
                print('autonomous switching cooldown not finished')
            else:
                self.auto_cooldown = 100
                autonomous = not autonomous
        elif key == ord('1'):
            upper_height()
        elif key == ord('2'):
            lower_height()
        elif key == ord('3'):
            above_basket()
        elif key == ord('R'):
            global gripper_status
            gripper_status = 'closed'


def ease_out_exp(x: float) -> float:
    x = min(max(x, 0), 1)
    return 1 - np.power(2.0, -10 * x)


def ease_out_quad(x: float) -> float:
    x = min(max(x, 0), 1)
    return 1 - (1 - x) * (1 - x)


class IKController:
    """Inverse Kinematics Controller"""

    def __init__(self):
        self.target_pos = None

    def target_reached(self):
        if self.target_pos is None:
            return True

        if len(self.waypoints) == 0:
            return True

        dist = np.linalg.norm([
            loc.pose_x - self.target_pos[0],
            loc.pose_y - self.target_pos[1],
        ])

        return dist < POSITION_THRESHOLD

    def set_target(self, target_pos: list[float]):
        if target_pos is None:
            self.waypoints = []
        else:
            logger.warning('pathfinding disabled')
            self.waypoints = [target_pos]  # navigation.plan_path(target_pos)
        self.target_pos = target_pos

    def get_target(self) -> list[float]:
        if self.waypoints is None or self.target_pos is None:
            return [0, 0]
        if len(self.waypoints) == 0:
            return [0, 0]
        return self.waypoints[0]

    def calculate_position_error(self):
        target_x, target_y = self.get_target()
        return np.sqrt((loc.pose_x - target_x)**2 + (loc.pose_y - target_y)**2)

    def calculate_bearing_error(self):
        target_x, target_y = self.get_target()
        dx, dy = target_x - loc.pose_x, target_y - loc.pose_y
        theta = np.arctan2(dy, dx)
        alpha = theta - loc.pose_theta
        if alpha > np.pi:
            # prevent robot from trying to turn 270 deg left instead of right
            alpha = alpha - 2 * np.pi
        if alpha < -np.pi:
            # prevent robot from trying to turn 270 deg right instead of left
            alpha = alpha + 2 * np.pi

        return alpha

    def update(self):
        if self.target_reached():
            return

        def inverse_kinematics(dx, theta):
            L = dx - theta * robot.AXLE_LENGTH / 2
            R = dx + theta * robot.AXLE_LENGTH / 2
            return L, R

        def balance_values(vL, vR):
            '''normalize vL, vR to -1 to 1'''
            max_value = max(abs(vL), abs(vR))
            vR = vR / max_value
            vL = vL / max_value
            return vL, vR

        rho = self.calculate_position_error()
        alpha = self.calculate_bearing_error()

        if rho < POSITION_THRESHOLD:
            self.waypoints = self.waypoints[1:]
            return

        # STEP 2: Controller
        # set min rho to limit influence on IK for large values of rho
        dx = 1 * min(rho, 2)
        dtheta = 10 * alpha

        # STEP 3: Compute wheelspeeds
        vL, vR = inverse_kinematics(dx, dtheta)
        # normalize values between -1 to 1
        vL, vR = balance_values(vL, vR)

        # convert to rotational vel
        # use `ease_out_quad` is robot is overshooting target
        ease_factor = ease_out_quad(rho)
        vL = vL * robot.MAX_SPEED * ease_factor
        vR = vR * robot.MAX_SPEED * ease_factor

        nav_logger.debug('==== NAVIGATION FRAME ===')
        nav_logger.debug(
            f'[pose_x = {loc.pose_x:.3}, pose_y = {loc.pose_y:.3}, pose_theta = {loc.pose_theta:.3}]')
        nav_logger.debug(f'target = {self.get_target()}')
        nav_logger.debug(f'rho = {rho}')
        nav_logger.debug(f'alpha = {alpha}')
        nav_logger.debug(f'dx = {dx}')
        nav_logger.debug(f'vL = {vL}')
        nav_logger.debug(f'vR = {vR}')

        # Normalize wheelspeed
        # (Keep the wheel speeds a bit less than the actual platform MAX_SPEED to minimize jerk)
        wheels.vL, wheels.vR = vL, vR


gripper_status = "closed"

wheels = None
grippers = None
keyboard_controller = None
ik_controller = None
task_root = task_tree.create_root()


def upper_height():
    # object is always on shelf 2 or 3
    robot.robot_parts['arm_1_joint'].setPosition(np.pi / 2)
    robot.robot_parts['arm_2_joint'].setPosition(1)  # -1.5 - 1.02
    robot.robot_parts['arm_3_joint'].setPosition(0)  # 1.5
    robot.robot_parts['arm_4_joint'].setPosition(1)
    robot.robot_parts['arm_5_joint'].setPosition(0)
    robot.robot_parts['arm_6_joint'].setPosition(0)
    robot.robot_parts['arm_7_joint'].setPosition(np.pi / 2)
    robot.robot_parts['torso_lift_joint'].setPosition(0.16)  # range 0 - 0.35


def lower_height():
    # object is always on shelf 2 or 3
    robot.robot_parts['arm_1_joint'].setPosition(np.pi / 2)
    robot.robot_parts['arm_2_joint'].setPosition(-0.7)  # -1.5 - 1.02
    robot.robot_parts['arm_3_joint'].setPosition(-np.pi / 2)  # 1.5
    robot.robot_parts['arm_4_joint'].setPosition(0)
    robot.robot_parts['arm_5_joint'].setPosition(0)
    robot.robot_parts['arm_6_joint'].setPosition(0.7)
    robot.robot_parts['arm_7_joint'].setPosition(0)
    robot.robot_parts['torso_lift_joint'].setPosition(0.35)  # range 0 - 0.35


def above_basket():
    robot.robot_parts['arm_1_joint'].setPosition(0.8)
    robot.robot_parts['arm_2_joint'].setPosition(-0.3)  # -1.5 - 1.02
    robot.robot_parts['arm_3_joint'].setPosition(-np.pi / 2)  # 1.5
    robot.robot_parts['arm_4_joint'].setPosition(2)
    robot.robot_parts['arm_5_joint'].setPosition(-2)
    robot.robot_parts['arm_6_joint'].setPosition(1.39)
    robot.robot_parts['arm_7_joint'].setPosition(np.pi / 2)
    robot.robot_parts['torso_lift_joint'].setPosition(0)  # range 0 - 0.35


def init():
    """Initialize manipulation module"""
    global wheels, grippers, keyboard_controller, ik_controller
    wheels = WheelMotors()
    grippers = Grippers()
    keyboard_controller = ManualController()
    ik_controller = IKController()

    above_basket()


def update():
    """Update hook for manipulation module"""

    task_root.tick_once()

    keyboard_controller.update()

    global gripper_status
    if autonomous:
        ik_controller.update()

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
