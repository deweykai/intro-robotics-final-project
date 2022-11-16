import py_trees as pyt
from py_trees.common import Status
import localization as loc
import numpy as np
import manipulation


class DriveTo(pyt.behaviour.Behaviour):
    def __init__(self, target_pos='', name='Drive To'):
        super().__init__(name)
        self.target_pos = target_pos

    def initialise(self):
        print(f'Set Target to {self.target_pos}')

    def update(self):
        dist = np.linalg.norm([
            loc.pose_x - self.target_pos[0],
            loc.pose_y - self.target_pos[1],
        ])

        if manipulation.ik_controller.target_reached():
            manipulation.ik_controller.set_target(None)
            return Status.SUCCESS
        else:
            manipulation.ik_controller.set_target(self.target_pos)
            return Status.RUNNING
