import py_trees as pyt
from py_trees.common import Status
import numpy as np
import logging
import bus
from utils.ease_func import ease_out_exp, ease_out_quad

logger = logging.getLogger(__name__)

cmd_gripper = bus.Publisher('/bot/cmd_gripper', bool)


class SetGripper(pyt.behaviour.Behaviour):
    def __init__(self, open_state, name='set gripper'):
        super().__init__(name)
        self.open_state = open_state

    def update(self):
        cmd_gripper.publish(self.open_state)
        return Status.SUCCESS
