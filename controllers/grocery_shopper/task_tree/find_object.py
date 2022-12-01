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
            dx = pos[0]
            dy = pos[1]

            dist = np.linalg.norm([dx, dy])
            if dist > 5 and dist < 10:
                # index 0 is object position
                object_location = [dx + loc.pose_x, dy + loc.pose_y]
                logger.info(f'target found at {object_location}')
                return Status.SUCCESS

        # find tree
        return Status.FAILURE
