import os
import sys
from typing import Tuple

import gym
import numpy as np

from pgdrive.envs.observation_type import ObservationType
from pgdrive.scene_creator.ego_vehicle.base_vehicle import BaseVehicle
from pgdrive.utils import import_pygame
from pgdrive.utils.constans import Decoration
from pgdrive.world.constants import PG_EDITION
from pgdrive.world.top_down_observation.top_down_obs_impl import WorldSurface, ObservationWindow, COLOR_BLACK, \
    VehicleGraphics, LaneGraphics

pygame = import_pygame()


class TopDownObservation(ObservationType):
    """
    Most of the source code is from Highway-Env, we only optimize and integrate it in PGDrive
    See more information on its Github page: https://github.com/eleurent/highway-env
    """
    RESOLUTION = (200, 200)  # pix x pix
    MAP_RESOLUTION = (2000, 2000)  # pix x pix
    MAX_RANGE = (50, 50)  # maximum detection distance = 50 M

    def __init__(self, vehicle_config, env, clip_rgb: bool):
        super(TopDownObservation, self).__init__(vehicle_config, env)
        self.rgb_clip = clip_rgb
        self.num_stacks = 3

        # self.obs_shape = (64, 64)
        self.obs_shape = self.RESOLUTION

        self.pygame = import_pygame()

        self.onscreen = env.config["use_render"]
        main_window_position = (0, 0)

        self._center_pos = None  # automatically change, don't set the value
        self._should_draw_map = True
        self._canvas_to_display_scaling = 0.0

        # scene
        self.road_network = None
        self.scene_manager = None

        # initialize
        pygame.init()
        pygame.display.set_caption(PG_EDITION + " (Top-down)")
        # main_window_position means the left upper location.
        os.environ['SDL_VIDEO_WINDOW_POS'] = '{},{}' \
            .format(main_window_position[0] - self.RESOLUTION[0], main_window_position[1])
        # Used for display only!
        self.screen = pygame.display.set_mode(
            (self.RESOLUTION[0] * 2, self.RESOLUTION[1] * 2)
        ) if self.onscreen else None

        # canvas
        self.init_canvas()
        self.init_obs_window()

    def init_obs_window(self):
        self.obs_window = ObservationWindow(self.MAX_RANGE, self.RESOLUTION)

    def init_canvas(self):
        self.canvas_runtime = WorldSurface(self.MAP_RESOLUTION, 0, pygame.Surface(self.MAP_RESOLUTION))
        self.canvas_background = WorldSurface(self.MAP_RESOLUTION, 0, pygame.Surface(self.MAP_RESOLUTION))

    def reset(self, env):
        self.scene_manager = env.scene_manager
        self.road_network = env.current_map.road_network
        self._should_draw_map = True

    def render(self) -> np.ndarray:
        if self.onscreen:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        sys.exit()

        if self._should_draw_map:
            self.draw_map()

        self.draw_scene()

        if self.onscreen:
            self.screen.fill(COLOR_BLACK)
            pygame.transform.scale2x(self.obs_window.get_observation_window(), self.screen)
            pygame.display.flip()

    def get_screenshot(self, name="screenshot.jpg"):
        pygame.image.save(self.screen, name)

    def draw_map(self) -> pygame.Surface:
        """
        :return: a big map surface, clip  and rotate to use a piece of it
        """
        surface = self.canvas_background

        # Setup the maximize size of the canvas
        # scaling and center can be easily found by bounding box
        b_box = self.road_network.get_bounding_box()
        self.canvas_background.set_colorkey(self.canvas_background.BLACK)
        x_len = b_box[1] - b_box[0]
        y_len = b_box[3] - b_box[2]
        max_len = max(x_len, y_len) + 20  # Add more 20 meters
        scaling = self.MAP_RESOLUTION[1] / max_len - 0.1
        assert scaling > 0

        # real-world distance * scaling = pixel in canvas
        self.canvas_background.scaling = scaling
        self.canvas_runtime.scaling = scaling
        # self._scaling = scaling

        centering_pos = ((b_box[0] + b_box[1]) / 2, (b_box[2] + b_box[3]) / 2)
        # self._center_pos = centering_pos
        self.canvas_runtime.move_display_window_to(centering_pos)

        surface.move_display_window_to(centering_pos)
        for _from in self.road_network.graph.keys():
            decoration = True if _from == Decoration.start else False
            for _to in self.road_network.graph[_from].keys():
                for l in self.road_network.graph[_from][_to]:
                    two_side = True if l is self.road_network.graph[_from][_to][-1] or decoration else False
                    LaneGraphics.LANE_LINE_WIDTH = 0.5
                    LaneGraphics.display(l, self.canvas_background, two_side)

        self.obs_window.reset(self.canvas_runtime)

        self._should_draw_map = False

    def draw_scene(self):
        # Set the active area that can be modify to accelerate
        pos = self.canvas_runtime.pos2pix(*self.scene_manager.ego_vehicle.position)
        clip_size = (int(self.obs_window.get_size()[0] * 1.1), int(self.obs_window.get_size()[0] * 1.1))
        self.canvas_runtime.set_clip((pos[0] - clip_size[0] / 2, pos[1] - clip_size[1] / 2, clip_size[0], clip_size[1]))
        self.canvas_runtime.fill(COLOR_BLACK)
        self.canvas_runtime.blit(self.canvas_background, (0, 0))

        # Draw vehicles
        # TODO PZH: I hate computing these in pygame-related code!!!
        ego_heading = self.scene_manager.ego_vehicle.heading_theta
        ego_heading = ego_heading if abs(ego_heading) > 2 * np.pi / 180 else 0

        VehicleGraphics.display(
            vehicle=self.scene_manager.ego_vehicle,
            surface=self.canvas_runtime,
            heading=ego_heading,
            color=VehicleGraphics.GREEN
        )
        for v in self.scene_manager.traffic_mgr.vehicles:
            if v is self.scene_manager.ego_vehicle:
                continue
            h = v.heading
            h = h if abs(h) > 2 * np.pi / 180 else 0
            VehicleGraphics.display(vehicle=v, surface=self.canvas_runtime, heading=h, color=VehicleGraphics.BLUE)

        # Prepare a runtime canvas for rotation
        return self.obs_window.render(
            canvas=self.canvas_runtime, position=pos, heading=self.scene_manager.ego_vehicle.heading_theta
        )

    @staticmethod
    def blit_rotate(
        surf: pygame.SurfaceType,
        image: pygame.SurfaceType,
        pos,
        angle: float,
    ) -> Tuple:
        """Many thanks to https://stackoverflow.com/a/54714144."""
        # calculate the axis aligned bounding box of the rotated image
        w, h = image.get_size()
        box = [pygame.math.Vector2(p) for p in [(0, 0), (w, 0), (w, -h), (0, -h)]]
        box_rotate = [p.rotate(angle) for p in box]
        min_box = (min(box_rotate, key=lambda p: p[0])[0], min(box_rotate, key=lambda p: p[1])[1])
        max_box = (max(box_rotate, key=lambda p: p[0])[0], max(box_rotate, key=lambda p: p[1])[1])

        # calculate the translation of the pivot
        origin_pos = w / 2, h / 2
        pivot = pygame.math.Vector2(origin_pos[0], -origin_pos[1])
        pivot_rotate = pivot.rotate(angle)
        pivot_move = pivot_rotate - pivot

        # calculate the upper left origin of the rotated image
        origin = (
            pos[0] - origin_pos[0] + min_box[0] - pivot_move[0], pos[1] - origin_pos[1] - max_box[1] + pivot_move[1]
        )
        # get a rotated image
        rotated_image = pygame.transform.rotate(image, angle)
        # rotate and blit the image
        surf.blit(rotated_image, origin)
        return origin

    def get_observation_window(self):
        return self.obs_window.get_observation_window()

    @property
    def observation_space(self):
        shape = self.obs_shape + (self.num_stacks, )
        if self.rgb_clip:
            return gym.spaces.Box(-0.0, 1.0, shape=shape, dtype=np.float32)
        else:
            return gym.spaces.Box(0, 255, shape=shape, dtype=np.uint8)

    def observe(self, vehicle: BaseVehicle):
        self.render()
        surface = self.get_observation_window()
        img = self.pygame.surfarray.array3d(surface)
        if self.rgb_clip:
            img = img.astype(np.float32) / 255
        else:
            img = img.astype(np.uint8)
        return np.transpose(img, (1, 0, 2))
