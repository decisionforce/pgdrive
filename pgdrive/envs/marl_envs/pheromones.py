import math

import numpy as np


class PheromoneMap:
    def __init__(self, total_width, total_length, num_channels=1, granularity=0.5):
        self.total_width = total_width
        self.total_length = total_length
        self.num_widths = int(math.ceil(total_width / granularity)) + 1
        self.num_lengths = int(math.ceil(total_length / granularity)) + 1
        self.num_channels = num_channels
        self.granularity = granularity
        self._map = np.zeros((self.num_widths, self.num_lengths, self.num_channels))

    def add(self, position, values):
        x, y = self.get_indices(position)
        if self.num_channels > 1:
            assert len(values) == self.num_channels
        else:
            assert np.isscalar(values)
        self._map[x, y] = values

    def get_indices(self, position):
        assert position[0] <= self.total_width
        assert position[1] <= self.total_length
        x = int(math.floor(position[0] / self.granularity))
        y = int(math.floor(position[1] / self.granularity))
        return x, y

    def clear(self):
        self._map.fill(0.0)

    def step(self):
        """
        Update for one step.
        """


if __name__ == '__main__':
    m = PheromoneMap(1, 1)
    m.add((1, 1), 999)

    mm = m._map[..., 0]
    print(m._map)
