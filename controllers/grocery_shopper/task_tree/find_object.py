import py_trees as pyt
from py_trees.common import Status
import logging
import numpy as np
import bus
from utils.object_to_world import object_to_world

logger = logging.getLogger(__name__)
# logger.setLevel(logging.DEBUG)

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

    for obj in objects:
        world_pos = object_to_world(
            [pose_x, pose_y, pose_theta], obj.position)

        logger.debug(world_pos)


class FindObject(pyt.behaviour.Behaviour):
    def __init__(self, close_range=False):
        super().__init__()
        self.close_range = close_range

    def update(self):
        global object_location

        for obj in detected_objects:
            dist = np.linalg.norm(obj.position)
            world_pos = object_to_world(
                [pose_x, pose_y, pose_theta], obj.position)

            # ignore objects on floor
            if world_pos[2] < -0.7:
                continue

            if not self.close_range:
                if dist > 5 and dist < 10:
                    object_location = world_pos

                    logger.info(f'target found at {object_location}')
                    return Status.SUCCESS

            if self.close_range:
                if dist < 3 and abs(pose_x - world_pos[0]) < 0.30 and dist > 1:
                    object_location = world_pos

                    logger.info(f'target found at {object_location}')
                    return Status.SUCCESS

        # find tree
        return Status.FAILURE
