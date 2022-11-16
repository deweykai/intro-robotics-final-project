import numpy as np

DISPLAY_DIM = 360

WORLD_MIN_X = -15
WORLD_MAX_X = 15
WORLD_MIN_Y = -8
WORLD_MAX_Y = 8


map_width = DISPLAY_DIM
map_height = DISPLAY_DIM

map_width = DISPLAY_DIM
map_height = DISPLAY_DIM
map_data = np.zeros((map_width, map_height))
dirty = False


def coords_world_to_map(pos):
    x, y = pos
    new_x = int((x / (WORLD_MAX_X - WORLD_MIN_X) + 0.5) * DISPLAY_DIM)
    new_y = int((y / (WORLD_MAX_X - WORLD_MIN_X) + 0.5) * DISPLAY_DIM)
    if new_x < 0 or new_x >= DISPLAY_DIM:
        raise Exception('x out of bounds')
    if new_y < 0 or new_y >= DISPLAY_DIM:
        raise Exception('y out of bounds')
    return new_x, new_y


def coords_map_to_world(map_pos):
    x, y = map_pos
    new_x = ((x / DISPLAY_DIM) - 0.5) * (WORLD_MAX_X - WORLD_MIN_X)
    new_y = ((y / DISPLAY_DIM) - 0.5) * (WORLD_MAX_X - WORLD_MIN_X)
    return new_x, new_y


class Mapper():
    def load(self):
        """Load raw map data from `raw_map.npy`"""

        print('Loading map...')
        self.raw_map = np.load('raw_map.npy')
        self.update_map_data()

    def update_map_data(self):
        """Process raw_map then save to map_data"""

        raw_map = self.raw_map.copy()

        global map_data, dirty
        map_data = (raw_map > 0.7) * 1
        dirty = True

mapper = Mapper()
mapper.load()
