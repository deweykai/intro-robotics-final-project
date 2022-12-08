import py_trees as pyt
from py_trees.common import Status
import numpy as np
import logging
import bus
#from utils.astar import plan_path
from utils.RRT import plan_path
import robot
from utils.ease_func import ease_out_exp, ease_out_quad

logger = logging.getLogger(__name__)
# logger.setLevel(logging.DEBUG)

POSITION_THRESHOLD = 0.1
left_wheel_pub = bus.Publisher('/bot/wheel/cmd_vel/left', float)
right_wheel_pub = bus.Publisher('/bot/wheel/cmd_vel/right', float)

pose_x, pose_y, pose_theta = 0, 0, 0


@bus.subscribe('/bot/pose', np.ndarray)
def gps_data(data):
    global pose_x, pose_y, pose_theta
    pose_x, pose_y, pose_theta = data


class IKController:
    """Inverse Kinematics Controller"""

    def __init__(self):
        self.target_pos = None

    def target_reached(self):
        if self.target_pos is None:
            return True

        if self.waypoints is None:
            return False

        if len(self.waypoints) == 0:
            return True

        dist = np.linalg.norm([
            pose_x - self.target_pos[0],
            pose_y - self.target_pos[1],
        ])

        return dist < POSITION_THRESHOLD

    def set_target(self, target_pos: list[float]):
        if target_pos is None:
            self.waypoints = []
        else:
            # logger.warning('pathfinding disabled')
            # self.waypoints = [target_pos]  # navigation.plan_path(target_pos)
            self.waypoints = plan_path([pose_x, pose_y], target_pos)
            if self.waypoints is not None and len(self.waypoints) > 1:
                self.waypoints = self.waypoints[1:]
        self.target_pos = target_pos

    def get_target(self) -> list[float]:
        if self.waypoints is None or self.target_pos is None:
            return [0, 0]
        if len(self.waypoints) == 0:
            return [0, 0]
        return self.waypoints[0]

    def calculate_position_error(self):
        target_x, target_y = self.get_target()
        return np.sqrt((pose_x - target_x)**2 + (pose_y - target_y)**2)

    def calculate_bearing_error(self):
        target_x, target_y = self.get_target()
        dx, dy = target_x - pose_x, target_y - pose_y
        theta = np.arctan2(dy, dx)
        alpha = theta - pose_theta
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

        logger.debug('==== NAVIGATION FRAME ===')
        logger.debug(
            f'[pose_x = {pose_x:.3}, pose_y = {pose_y:.3}, pose_theta = {pose_theta:.3}]')
        logger.debug(f'target = {self.get_target()}')
        logger.debug(f'rho = {rho}')
        logger.debug(f'alpha = {alpha}')
        logger.debug(f'dx = {dx}')
        logger.debug(f'vL = {vL}')
        logger.debug(f'vR = {vR}')

        # Normalize wheelspeed
        # (Keep the wheel speeds a bit less than the actual platform MAX_SPEED to minimize jerk)
        left_wheel_pub.publish(vL)
        right_wheel_pub.publish(vR)


ik_controller = IKController()


class DriveTo(pyt.behaviour.Behaviour):
    def __init__(self, get_position, name='Drive To'):
        super().__init__(name)
        self.get_position = get_position

    def initialise(self):
        self.target_pos = self.get_position()
        logger.info(f'Set Target to {self.target_pos}')
        ik_controller.set_target(self.target_pos)

    def update(self):
        dist = np.linalg.norm([
            pose_x - self.target_pos[0],
            pose_y - self.target_pos[1],
        ])

        ik_controller.update()

        if ik_controller.waypoints is None:
            return Status.FAILURE

        if ik_controller.target_reached():
            logger.info('target reached')
            ik_controller.set_target(None)
            return Status.SUCCESS
        else:
            return Status.RUNNING
