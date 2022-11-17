import numpy as np
import ikpy
from ikpy.chain import Chain

arm_chain = Chain.from_urdf_file('robot_urdf.urdf')
