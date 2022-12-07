import py_trees as pyt
from py_trees.common import Status
import logging
import numpy as np
import bus

logger = logging.getLogger(__name__)

object_location = [0, 0]

pose_x, pose_y, pose_theta = 0, 0, 0


@bus.subscribe('/bot/pose', np.ndarray)
def gps_data(data):
    global pose_x, pose_y, pose_theta
    pose_x, pose_y, pose_theta = data


detected_objects = []


@bus.subscribe('/bot/sensor/camera_rec', list)
def camera_data(objects):
    global detected_objects
    detected_objects = objects


class FindObject(pyt.behaviour.Behaviour):
    def update(self):
        global object_location

        positions = [o[0] for o in detected_objects]
        for pos in positions:
            dx = pos[0]
            dy = pos[1]

            dist = np.linalg.norm([dx, dy])
            if dist > 5 and dist < 10:
                # index 0 is object position
                object_location = [dx + pose_x, dy + pose_y]
                logger.info(f'target found at {object_location}')
                return Status.SUCCESS

        # find tree
        return Status.FAILURE
