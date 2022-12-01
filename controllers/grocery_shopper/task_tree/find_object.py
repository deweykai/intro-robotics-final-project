import py_trees as pyt
from py_trees.common import Status
import localization as loc
import vision
import logging
import numpy as np

logger = logging.getLogger(__name__)

object_location = [0, 0]


class FindObject(pyt.behaviour.Behaviour):
    def update(self):
        global object_location

        objects = vision.detect_filtered_objects()
        positions = [o[0] for o in objects]
        for pos in positions:
            dx = pos[0] - loc.pose_x
            dy = pos[1] - loc.pose_y
            if np.linalg.norm([dx, dy]) < 5:
                # index 0 is object position
                object_location = pos
                logger.info(f'target found at {pos}')
                return Status.SUCCESS

        # find tree
        return Status.FAILURE
