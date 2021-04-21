from typing import Optional, Union, Iterable

import cv2
import numpy as np
from pgdrive.constants import Decoration
from pgdrive.obs.top_down_obs_impl import WorldSurface, VehicleGraphics, LaneGraphics
from pgdrive.utils.utils import import_pygame

pygame = import_pygame()


def draw_top_down_map(map, resolution: Iterable = (512, 512), simple_draw=True, return_surface=False) -> Optional[
    Union[np.ndarray, pygame.Surface]]:
    surface = WorldSurface(map.film_size, 0, pygame.Surface(map.film_size))
    b_box = map.road_network.get_bounding_box()
    x_len = b_box[1] - b_box[0]
    y_len = b_box[3] - b_box[2]
    max_len = max(x_len, y_len)
    # scaling and center can be easily found by bounding box
    scaling = map.film_size[1] / max_len - 0.1
    surface.scaling = scaling
    centering_pos = ((b_box[0] + b_box[1]) / 2, (b_box[2] + b_box[3]) / 2)
    surface.move_display_window_to(centering_pos)
    for _from in map.road_network.graph.keys():
        decoration = True if _from == Decoration.start else False
        for _to in map.road_network.graph[_from].keys():
            for l in map.road_network.graph[_from][_to]:
                if simple_draw:
                    LaneGraphics.simple_draw(l, surface)
                else:
                    two_side = True if l is map.road_network.graph[_from][_to][-1] or decoration else False
                    LaneGraphics.display(l, surface, two_side)

    if return_surface:
        return surface
    ret = cv2.resize(pygame.surfarray.pixels_red(surface), resolution, interpolation=cv2.INTER_LINEAR)
    return ret


class TopDownRenderer:
    def __init__(self, map):
        self._map = map
        self._background = draw_top_down_map(map, simple_draw=False, return_surface=True)
        self._size = tuple(self._background.get_size())
        self._screen = pygame.display.set_mode(self._size)
        self._screen.blit(self._background)

        self._screen.fill(color_white)

    def render(self, vehicles):
        self._screen.blit(self._background)
        for v in vehicles:
            h = v.heading
            h = h if abs(h) > 2 * np.pi / 180 else 0
            VehicleGraphics.display(vehicle=v, surface=self._screen, heading=h, color=VehicleGraphics.BLUE)
