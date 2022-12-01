import py_trees as pyt
from py_trees.common import Status
import localization as loc
import numpy as np
import manipulation
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


class FaceTowards(pyt.behaviour.Behaviour):
    def __init__(self, get_target=lambda: [0, 0], name='face towards'):
        super().__init__(name)
        self.get_target = get_target

    def update(self):
        target = self.get_target()
        relative_target = [target[0] - loc.pose_x, target[1] - loc.pose_y]

        relative_bearing = np.arctan2(relative_target[1], relative_target[0])
        bearing_error = relative_bearing - loc.pose_theta

        logger.debug('=== FACE TOWARDS ===')
        logger.debug(f'Relative Bearing: {relative_bearing}')
        logger.debug(f'Bearing Error: {bearing_error}')

        if np.abs(bearing_error) < 0.01:
            return Status.SUCCESS

        speed = manipulation.ease_out_quad(np.abs(bearing_error)) * 5

        if bearing_error > 0:
            manipulation.wheels.vL = -speed
            manipulation.wheels.vR = speed
        else:
            manipulation.wheels.vL = speed
            manipulation.wheels.vR = -speed

        return Status.RUNNING
