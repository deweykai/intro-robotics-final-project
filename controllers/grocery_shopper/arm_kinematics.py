import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import math
import logging

logger = logging.getLogger(__name__)
# logger.setLevel(logging.DEBUG)

arm_chain = Chain.from_urdf_file("robot_urdf.urdf")
print(arm_chain.links)


def set_arm_to_target(target):
    offset_target = [
        -target[2] + 0.22,
        -target[0] + 0.08,
        target[1] + 0.97 + 0.2
    ]
    initial_position = [0, 0, 0, 0]
    # [m.getPositionSensor().getValue()
    #                     for m in motors] + [0, 0, 0, 0]
    # )

    ikResults = arm_chain.inverse_kinematics(
        target
    )

    logger.warning('setting robot positions disabled')
    # for res in range(len(ikResults)):
    #     if arm_chain.links[res].name in part_names:
    #         robot.getDevice(arm_chain.links[res].name).setPosition(
    #             ikResults[res])
    #         print("Setting {} to {}".format(
    #             arm_chain.links[res].name, ikResults[res]))

    import ikpy.utils.plot as plot_utils
    import matplotlib.pyplot as plt

    fig, ax = plot_utils.init_3d_figure()

    arm_chain.plot(ikResults, ax, target=target)
    plt.show()


set_arm_to_target([1, 0.5, 1])
