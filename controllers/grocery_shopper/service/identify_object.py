import numpy as np
import bus
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


detected_objects = []


@bus.subscribe('/bot/task/detect_object', np.ndarray)
def detect_object(object_location):
    object_id = -1
    for i, ref_object in enumerate(detected_objects):
        if np.linalg.norm(ref_object - object_location) < 0.4:
            object_id = i
            logger.info(f'Detected object #{object_id+1}')

    if object_id == -1:
        object_id = len(detected_objects)
        detected_objects.append(object_location)
        logger.info(f'Registering object #{object_id+1} at {object_location}')
