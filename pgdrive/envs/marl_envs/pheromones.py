import math

import numpy as np


class PheromoneMap:
    def __init__(
        self, total_width, total_length, num_channels=1, granularity=0.5, attenuation_rate=1.0, diffusion_rate=1.0
    ):
        self.total_width = total_width
        self.total_length = total_length
        self.num_widths = int(math.ceil(total_width / granularity)) + 1
        self.num_lengths = int(math.ceil(total_length / granularity)) + 1
        self.num_channels = num_channels
        self.granularity = granularity

        assert 0.0 <= attenuation_rate <= 1.0
        self.attenuation_rate = attenuation_rate

        assert 0.0 <= diffusion_rate <= 1.0
        self.diffusion_rate = diffusion_rate

        self._map = np.zeros((self.num_widths, self.num_lengths, self.num_channels))
        self._tmp_map = np.zeros((self.num_widths, self.num_lengths, self.num_channels))

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
        self._tmp_map.fill(0.0)

    def step(self):
        """This function should be called when environment was stepped!"""
        self.diffuse()
        self.attenuate()

    def attenuate(self):
        if self.attenuation_rate == 1.0:
            return
        self._map = self._map * self.attenuation_rate

    def diffuse(self):
        if self.diffusion_rate == 1.0:
            return
        self._tmp_map.fill(0.0)
        for x in range(1, self.num_widths - 1):
            for y in range(1, self.num_lengths - 1):
                val = self._map[x, y]
                if np.all(val == 0.0):
                    continue
                diffused_val = (val * (1 - self.diffusion_rate)) / 8
                self._tmp_map[[x - 1, x, x + 1], y - 1] += diffused_val
                self._tmp_map[[x - 1, x, x + 1], y + 1] += diffused_val
                self._tmp_map[[x - 1, x + 1], y] += diffused_val
                self._tmp_map[x, y] += val * self.diffusion_rate

        # in-place replacement
        self._map[...] = self._tmp_map


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    # ===== Visualize =====
    l = 10
    m = PheromoneMap(l, l, attenuation_rate=0.99, diffusion_rate=0.99)
    filenames = []
    m.add((l / 2, l / 2), 1)
    for i in range(100):
        m.step()
        plt.imshow(m._map[..., 0], vmin=0, vmax=1, cmap="inferno")
        plt.colorbar()
        plt.title("Timesteps: {}".format(i))
        plt.show()

    # ===== Generate Video =====
    import os
    import imageio

    l = 10
    m = PheromoneMap(l, l, attenuation_rate=0.99, diffusion_rate=0.99)
    filenames = []
    m.add((l / 2, l / 2), 1)
    for i in range(100):
        m.step()
        plt.imshow(m._map[..., 0], vmin=0, vmax=1, cmap="inferno")
        plt.colorbar()
        plt.title("Timesteps: {}".format(i))
        # plt.show()
        filename = f'{i}.png'
        filenames.append(filename)
        plt.savefig(filename)
        plt.close()
        if (i + 1) % 10 == 0:
            print("Finish {} steps!".format(i + 1))

    # build gif
    with imageio.get_writer('mygif.gif', mode='I') as writer:
        for filename in filenames:
            image = imageio.imread(filename)
            writer.append_data(image)

    # Remove files
    for filename in set(filenames):
        os.remove(filename)
