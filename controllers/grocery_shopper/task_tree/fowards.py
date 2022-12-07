import py_trees as pyt
from py_trees.common import Status
import logging
import bus
from robot import timestep
from utils.ease_func import ease_out_exp, ease_out_quad

logger = logging.getLogger(__name__)

left_wheel_pub = bus.Publisher('/bot/wheel/cmd_vel/left', float)
right_wheel_pub = bus.Publisher('/bot/wheel/cmd_vel/right', float)


class DriveForwards(pyt.behaviour.Behaviour):
    def __init__(self, speed=1.0, name='drive straight', timeout_ms=3_000, cond=None):
        super().__init__(name)
        self.timeout_ms = timeout_ms
        self.speed = speed
        self.cond = cond

    def initialise(self):
        self.time = 0

    def update(self):
        left_wheel_pub.publish(self.speed)
        right_wheel_pub.publish(self.speed)

        self.time += timestep
        if self.time > self.timeout_ms:
            return Status.SUCCESS

        if self.cond is not None and not self.cond():
            return Status.RUNNING
        return Status.SUCCESS
