import py_trees as pyt
from py_trees.common import Status
import numpy as np
import logging
import bus
from utils.ease_func import ease_out_exp, ease_out_quad

logger = logging.getLogger(__name__)

cmd_arm = bus.Publisher('/bot/cmd_arm', str)


class SetArms(pyt.behaviour.Behaviour):
    def __init__(self, get_target=lambda: [0, 0], state=None, name='set arms'):
        super().__init__(name)
        self.get_target = get_target
        self.state = state

    def update(self):
        if self.state is not None:
            cmd_arm.publish(self.state)
            return Status.SUCCESS

        _, _, z = self.get_target()
        if z > -0.10:
            # upper shelf
            cmd_arm.publish('upper')
            return Status.SUCCESS
        else:
            # lower shelf
            cmd_arm.publish('lower')
            return Status.SUCCESS

        return Status.FAILURE
