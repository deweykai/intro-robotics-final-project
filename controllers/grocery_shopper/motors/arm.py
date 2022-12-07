import robot
import numpy as np
import bus
import logging

logger = logging.getLogger(__name__)

arm_parts = ["arm_1_joint", "arm_2_joint",  "arm_3_joint",  "arm_4_joint",
             "arm_5_joint", "arm_6_joint",  "arm_7_joint"]

for part in arm_parts:
    max_vel = robot.robot_parts[part].getMaxVelocity()
    robot.robot_parts[part].setVelocity(max_vel / 5.0)

def upper_height():
    # object is always on shelf 2 or 3
    robot.robot_parts['arm_1_joint'].setPosition(np.pi / 2)
    robot.robot_parts['arm_2_joint'].setPosition(1)  # -1.5 - 1.02
    robot.robot_parts['arm_3_joint'].setPosition(0)  # 1.5
    robot.robot_parts['arm_4_joint'].setPosition(1)
    robot.robot_parts['arm_5_joint'].setPosition(0)
    robot.robot_parts['arm_6_joint'].setPosition(0)
    robot.robot_parts['arm_7_joint'].setPosition(np.pi / 2)
    robot.robot_parts['torso_lift_joint'].setPosition(0.16)  # range 0 - 0.35


def lower_height():
    # object is always on shelf 2 or 3
    robot.robot_parts['arm_1_joint'].setPosition(np.pi / 2)
    robot.robot_parts['arm_2_joint'].setPosition(-0.7)  # -1.5 - 1.02
    robot.robot_parts['arm_3_joint'].setPosition(-np.pi / 2)  # 1.5
    robot.robot_parts['arm_4_joint'].setPosition(0)
    robot.robot_parts['arm_5_joint'].setPosition(0)
    robot.robot_parts['arm_6_joint'].setPosition(0.7)
    robot.robot_parts['arm_7_joint'].setPosition(0)
    robot.robot_parts['torso_lift_joint'].setPosition(0.35)  # range 0 - 0.35


def above_basket():
    robot.robot_parts['arm_1_joint'].setPosition(0.8)
    robot.robot_parts['arm_2_joint'].setPosition(-0.3)  # -1.5 - 1.02
    robot.robot_parts['arm_3_joint'].setPosition(-np.pi / 2)  # 1.5
    robot.robot_parts['arm_4_joint'].setPosition(2)
    robot.robot_parts['arm_5_joint'].setPosition(-2)
    robot.robot_parts['arm_6_joint'].setPosition(1.39)
    robot.robot_parts['arm_7_joint'].setPosition(np.pi / 2)
    robot.robot_parts['torso_lift_joint'].setPosition(0)  # range 0 - 0.35


@bus.subscribe('/bot/cmd_arm', str)
def cmd_arm(position):
    if position == 'upper':
        upper_height()
    elif position == 'lower':
        lower_height()
    elif position == 'basket':
        above_basket()
    else:
        logger.error('unknown arm position')
