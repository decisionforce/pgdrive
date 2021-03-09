import numpy as np

from pgdrive.scene_creator.ego_vehicle.base_vehicle import BaseVehicle
from pgdrive.utils import import_pygame
from pgdrive.utils.constans import Decoration
from pgdrive.world.top_down_observation.top_down_obs_impl import WorldSurface, COLOR_BLACK, \
    VehicleGraphics, LaneGraphics, ObservationWindowMultiChannel
from pgdrive.world.top_down_observation.top_down_observation import TopDownObservation

pygame = import_pygame()


class TopDownMultiChannel(TopDownObservation):
    """
    Most of the source code is from Highway-Env, we only optimize and integrate it in PGDrive
    See more information on its Github page: https://github.com/eleurent/highway-env
    """
    RESOLUTION = (200, 200)  # pix x pix
    MAP_RESOLUTION = (2000, 2000)  # pix x pix
    MAX_RANGE = (50, 50)  # maximum detection distance = 50 M

    CHANNEL_NAMES = ["road_network", "traffic_flow", "target_vehicle", "navigation"]

    def __init__(self, vehicle_config, env, clip_rgb: bool):
        super(TopDownMultiChannel, self).__init__(vehicle_config, env, clip_rgb)
        # self.rgb_clip = clip_rgb
        # self.num_stacks = 3
        #
        # # self.obs_shape = (64, 64)
        # self.obs_shape = self.RESOLUTION
        #
        # self.pygame = import_pygame()
        #
        # self.onscreen = env.config["use_render"]
        # main_window_position = (0, 0)
        #
        # self._center_pos = None  # automatically change, don't set the value
        # self._should_draw_map = True
        # self._canvas_to_display_scaling = 0.0
        #
        # # scene
        # self.road_network = None
        # self.scene_manager = None
        #
        # # initialize
        # pygame.init()
        # pygame.display.set_caption(PG_EDITION + " (Top-down)")
        # # main_window_position means the left upper location.
        # os.environ['SDL_VIDEO_WINDOW_POS'] = '{},{}' \
        #     .format(main_window_position[0] - self.RESOLUTION[0], main_window_position[1])
        # # Used for display only!
        # self.screen = pygame.display.set_mode(
        #     (self.RESOLUTION[0] * 2, self.RESOLUTION[1] * 2)) if self.onscreen else None
        #
        # # canvas
        # self.canvas_background = WorldSurface(self.MAP_RESOLUTION, 0, pygame.Surface(self.MAP_RESOLUTION))
        # self.canvas_navigation = WorldSurface(self.MAP_RESOLUTION, 0, pygame.Surface(self.MAP_RESOLUTION))
        # self.canvas_surrounding = WorldSurface(self.MAP_RESOLUTION, 0, pygame.Surface(self.MAP_RESOLUTION))
        #
        # self.obs_window = ObservationWindow(self.MAX_RANGE, self.RESOLUTION)

    def init_obs_window(self):
        self.obs_window = ObservationWindowMultiChannel(self.CHANNEL_NAMES, self.MAX_RANGE, self.RESOLUTION)
        # self.obs_window  = dict(
        #     road_network=ObservationWindow(self.MAX_RANGE, self.RESOLUTION),
        #     traffic_flow=ObservationWindow(self.MAX_RANGE, self.RESOLUTION),
        #     target_vehicle=ObservationWindow(self.MAX_RANGE, self.RESOLUTION),
        #     navigation=ObservationWindow(self.MAX_RANGE, self.RESOLUTION),
        # )

    def init_canvas(self):
        self.canvas_background = WorldSurface(self.MAP_RESOLUTION, 0, pygame.Surface(self.MAP_RESOLUTION))
        self.canvas_navigation = WorldSurface(self.MAP_RESOLUTION, 0, pygame.Surface(self.MAP_RESOLUTION))
        self.canvas_runtime = WorldSurface(self.MAP_RESOLUTION, 0, pygame.Surface(self.MAP_RESOLUTION))

    def reset(self, env):
        self.scene_manager = env.scene_manager
        self.road_network = env.current_map.road_network
        self.target_vehicle = env.vehicle
        self._should_draw_map = True

    # def render(self) -> np.ndarray:
    #     if self.onscreen:
    #         for event in pygame.event.get():
    #             if event.type == pygame.KEYDOWN:
    #                 if event.key == pygame.K_ESCAPE:
    #                     sys.exit()
    #
    #     if self._should_draw_map:
    #         self.draw_map()
    #
    #     self.draw_scene()
    #
    #     if self.onscreen:
    #         self.screen.fill(COLOR_BLACK)
    #         pygame.transform.scale2x(self.obs_window.get_observation_window(), self.screen)
    #         pygame.display.flip()

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
        self.canvas_navigation.scaling = scaling
        # self._scaling = scaling

        centering_pos = ((b_box[0] + b_box[1]) / 2, (b_box[2] + b_box[3]) / 2)
        # self._center_pos = centering_pos
        self.canvas_runtime.move_display_window_to(centering_pos)
        self.canvas_navigation.move_display_window_to(centering_pos)

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

        self.draw_navigation()

    def draw_scene(self):
        # Set the active area that can be modify to accelerate
        pos = self.canvas_runtime.pos2pix(*self.scene_manager.ego_vehicle.position)
        clip_size = (int(self.obs_window.get_size()[0] * 1.1), int(self.obs_window.get_size()[0] * 1.1))
        self.canvas_runtime.set_clip((pos[0] - clip_size[0] / 2, pos[1] - clip_size[1] / 2, clip_size[0], clip_size[1]))
        self.canvas_runtime.fill(COLOR_BLACK)

        self.canvas_runtime.blit(self.canvas_background, (0, 0))
        # self.canvas_runtime.blit(self.canvas_navigation, (0, 0))

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

        return self.obs_window.render(
            canvas_dict=dict(
                road_network=self.canvas_background,  # TODO
                traffic_flow=self.canvas_runtime,
                target_vehicle=self.canvas_runtime,  # TODO
                navigation=self.canvas_navigation,
            ), position=pos, heading=self.scene_manager.ego_vehicle.heading_theta
        )

    def get_observation_window(self):
        return self.obs_window.get_observation_window()

    def _transform(self, img):
        img = img[..., 0] > 0
        if self.rgb_clip:
            img = img.astype(np.float32)
        else:
            img = img.astype(np.uint8) * 255
        return img

    def observe(self, vehicle: BaseVehicle):
        self.render()
        surface_dict = self.get_observation_window()
        img_dict = {k: pygame.surfarray.array3d(surface) for k, surface in surface_dict.items()}

        # Gray scale
        img_dict = {k: self._transform(img) for k, img in img_dict.items()}

        # Reorder
        pass  # TODO

        # Stack
        img = np.stack(list(img_dict.values()), axis=2)
        return np.transpose(img, (1, 0, 2))
        # return np.transpose(img, (1, 0, 2))

    def draw_navigation(self):
        checkpoints = self.target_vehicle.routing_localization.checkpoints
        for i, c in enumerate(checkpoints[:-1]):
            lanes = self.road_network.graph[c][checkpoints[i + 1]]
            for lane in lanes:
                LaneGraphics.simple_draw(lane, self.canvas_navigation, color=(255, 0, 0))
