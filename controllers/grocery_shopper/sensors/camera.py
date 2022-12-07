from robot import camera
import bus
import logging
import numpy as np

logger = logging.getLogger(__name__)

color_ranges = []


def add_color_range_to_detect(lower_bound, upper_bound):
    """
    @param lower_bound: Tuple of BGR values
    @param upper_bound: Tuple of BGR values
    """
    global color_ranges
    logger.debug(f'Add [{lower_bound}, {upper_bound}] to detect')
    color_ranges.append(np.array([lower_bound, upper_bound]))


def check_if_color_in_range(bgr_tuple):
    """
    @param bgr_tuple: Tuple of BGR values
    @returns Boolean: True if bgr_tuple is in any of the color ranges specified in color_ranges
    """
    bgr_tuple = bgr_tuple[:3]
    for entry in color_ranges:
        lower, upper = entry[0], entry[1]
        if (bgr_tuple >= lower).all() and (bgr_tuple <= upper).all():
            return True
    return False


def detect_objects():
    objects = camera.getRecognitionObjects()
    rec = [(o.getPosition(), o.getColors()) for o in objects]
    return rec


def filter_colors(objects):
    filtered = list(filter(lambda c: check_if_color_in_range(c[1]), objects))
    return filtered


def detect_filtered_objects():
    return filter_colors(detect_objects())

pub_detected_objects = bus.Publisher('/bot/sensor/camera_rec', list)


@bus.subscribe('/bot/cmd_tick', int)
def update_camera(_):
    filtered_objects = detect_filtered_objects()
    pub_detected_objects.publish(filtered_objects)