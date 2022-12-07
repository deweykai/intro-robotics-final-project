import py_trees as pyt
from py_trees.common import Status
import numpy as np
from utils.ease_func import ease_out_quad
import logging
import bus

logger = logging.getLogger(__name__)
# logger.setLevel(logging.DEBUG)

pose_x, pose_y, pose_theta = 0, 0, 0


@bus.subscribe('/bot/pose', np.ndarray)
def gps_data(data):
    global pose_x, pose_y, pose_theta
    pose_x, pose_y, pose_theta = data


left_wheel_pub = bus.Publisher('/bot/wheel/cmd_vel/left', float)
right_wheel_pub = bus.Publisher('/bot/wheel/cmd_vel/right', float)


class FaceTowards(pyt.behaviour.Behaviour):
    def __init__(self, get_target=lambda: [0, 0], name='face towards'):
        super().__init__(name)
        self.get_target = get_target

    def update(self):
        target = self.get_target()
        relative_target = [target[0] - pose_x, target[1] - pose_y]

        relative_bearing = np.arctan2(relative_target[1], relative_target[0])
        bearing_error = relative_bearing - pose_theta

        logger.debug('=== FACE TOWARDS ===')
        logger.debug(f'Relative Bearing: {relative_bearing}')
        logger.debug(f'Bearing Error: {bearing_error}')

        if np.abs(bearing_error) < 0.005:
            left_wheel_pub.publish(0.0)
            right_wheel_pub.publish(0.0)
            return Status.SUCCESS

        speed = ease_out_quad(np.abs(bearing_error)) * 5.0

        if bearing_error > 0:
            left_wheel_pub.publish(speed * 0.1)
            right_wheel_pub.publish(speed)
        else:
            left_wheel_pub.publish(speed * 0.1)
            right_wheel_pub.publish(-speed)

        return Status.RUNNING
