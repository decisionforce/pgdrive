from typing import List, Tuple, Union

import numpy as np
from pgdrive.scene_creator.lane.abs_lane import LineType
from pgdrive.scene_creator.lane.circular_lane import CircularLane
from pgdrive.scene_creator.lane.straight_lane import StraightLane
from pgdrive.utils import import_pygame

PositionType = Union[Tuple[float, float], np.ndarray]
pygame = import_pygame()
COLOR_BLACK = pygame.Color("black")


class ObservationWindow:
    def __init__(self, max_range, resolution):
        self.max_range = max_range
        self.resolution = resolution
        self.receptive_field = None
        self.receptive_field_double = None

        self.canvas_rotate = None
        self.canvas_uncropped = pygame.Surface(
            (int(self.resolution[0] * np.sqrt(2)) + 1, int(self.resolution[1] * np.sqrt(2)) + 1)
        )

        self.canvas_display = pygame.Surface(self.resolution)
        self.canvas_display.fill(COLOR_BLACK)

    def reset(self, canvas_runtime):
        canvas_runtime.fill(COLOR_BLACK)

        # Assume max_range is only the radius!
        self.receptive_field_double = (
            int(canvas_runtime.pix(self.max_range[0] * np.sqrt(2))) * 2,
            int(canvas_runtime.pix(self.max_range[1] * np.sqrt(2))) * 2
        )
        self.receptive_field = (
            int(canvas_runtime.pix(self.max_range[0])) * 2, int(canvas_runtime.pix(self.max_range[1])) * 2
        )
        self.canvas_rotate = pygame.Surface(self.receptive_field_double)
        self.canvas_rotate.fill(COLOR_BLACK)
        self.canvas_display.fill(COLOR_BLACK)
        self.canvas_uncropped.fill(COLOR_BLACK)

    def _blit(self, canvas, position):
        self.canvas_rotate.blit(
            canvas, (0, 0), (
                position[0] - self.receptive_field_double[0] / 2, position[1] - self.receptive_field_double[1] / 2,
                self.receptive_field_double[0], self.receptive_field_double[1]
            )
        )

    def _rotate(self, heading):
        rotation = np.rad2deg(heading) + 90
        scale = self.canvas_uncropped.get_size()[0] / self.canvas_rotate.get_size()[0]
        return pygame.transform.rotozoom(self.canvas_rotate, rotation, scale)

    def _crop(self, new_canvas):
        size = self.canvas_display.get_size()
        self.canvas_display.blit(
            new_canvas,
            (0, 0),
            (
                new_canvas.get_size()[0] / 2 - size[0] / 2,  # Left
                new_canvas.get_size()[1] / 2 - size[1] / 2,  # Top
                size[0],  # Width
                size[1]  # Height
            )
        )

    def render(self, canvas, position, heading):
        # Prepare a runtime canvas for rotation. Assume max_range is only the radius, not diameter!
        self._blit(canvas, position)

        # Rotate the image so that ego is always heading top
        new_canvas = self._rotate(heading)

        # Crop the rotated image and then resize to the desired resolution
        self._crop(new_canvas)

        return self.canvas_display

    def get_observation_window(self):
        return self.canvas_display

    def get_size(self):
        assert self.canvas_rotate is not None
        return self.canvas_rotate.get_size()


class WorldSurface(pygame.Surface):
    """
    A pygame Surface implementing a local coordinate system so that we can move and zoom in the displayed area.
    From highway-env, See more information on its Github page: https://github.com/eleurent/highway-env.
    """

    BLACK = (0, 0, 0)
    GREY = (100, 100, 100)
    GREEN = (50, 200, 0)
    YELLOW = (200, 200, 0)
    WHITE = (255, 255, 255)
    INITIAL_SCALING = 5.5
    INITIAL_CENTERING = [0.5, 0.5]
    SCALING_FACTOR = 1.3
    MOVING_FACTOR = 0.1

    def __init__(self, size: Tuple[int, int], flags: object, surf: pygame.SurfaceType) -> None:
        surf.fill(pygame.Color("Black"))
        super().__init__(size, flags, surf)
        self.origin = np.array([0, 0])
        self.scaling = self.INITIAL_SCALING
        self.centering_position = self.INITIAL_CENTERING
        self.fill(self.BLACK)

    def pix(self, length: float) -> int:
        """
        Convert a distance [m] to pixels [px].

        :param length: the input distance [m]
        :return: the corresponding size [px]
        """
        return int(length * self.scaling)

    def pos2pix(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert two world coordinates [m] into a position in the surface [px]

        :param x: x world coordinate [m]
        :param y: y world coordinate [m]
        :return: the coordinates of the corresponding pixel [px]
        """
        return self.pix(x - self.origin[0]), self.pix(y - self.origin[1])

    def vec2pix(self, vec: PositionType) -> Tuple[int, int]:
        """
        Convert a world position [m] into a position in the surface [px].

        :param vec: a world position [m]
        :return: the coordinates of the corresponding pixel [px]
        """
        return self.pos2pix(vec[0], vec[1])

    def is_visible(self, vec: PositionType, margin: int = 50) -> bool:
        """
        Is a position visible in the surface?
        :param vec: a position
        :param margin: margins around the frame to test for visibility
        :return: whether the position is visible
        """
        x, y = self.vec2pix(vec)
        return -margin < x < self.get_width() + margin and -margin < y < self.get_height() + margin

    def move_display_window_to(self, position: PositionType) -> None:
        """
        Set the origin of the displayed area to center on a given world position.

        :param position: a world position [m]
        """
        self.origin = position - np.array(
            [
                self.centering_position[0] * self.get_width() / self.scaling,
                self.centering_position[1] * self.get_height() / self.scaling
            ]
        )

    def handle_event(self, event: pygame.event.EventType) -> None:
        """
        Handle pygame events for moving and zooming in the displayed area.

        :param event: a pygame event
        """
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_l:
                self.scaling *= 1 / self.SCALING_FACTOR
            if event.key == pygame.K_o:
                self.scaling *= self.SCALING_FACTOR
            if event.key == pygame.K_m:
                self.centering_position[0] -= self.MOVING_FACTOR
            if event.key == pygame.K_k:
                self.centering_position[0] += self.MOVING_FACTOR


class VehicleGraphics:
    RED = (255, 100, 100)
    GREEN = (50, 200, 0)
    BLUE = (100, 200, 255)
    YELLOW = (200, 200, 0)
    BLACK = (60, 60, 60)
    PURPLE = (200, 0, 150)
    DEFAULT_COLOR = YELLOW
    EGO_COLOR = GREEN
    font = None

    @classmethod
    def display(cls, vehicle, surface, color, heading, label: bool = False) -> None:
        """
        Display a vehicle on a pygame surface.

        The vehicle is represented as a colored rotated rectangle.

        :param vehicle: the vehicle to be drawn
        :param surface: the surface to draw the vehicle on
        :param label: whether a text label should be rendered
        """
        if not surface.is_visible(vehicle.position):
            return
        w = surface.pix(vehicle.WIDTH)
        h = surface.pix(vehicle.LENGTH)
        position = [*surface.pos2pix(vehicle.position[0], vehicle.position[1])]
        angle = np.rad2deg(heading)
        box = [pygame.math.Vector2(p) for p in [(-h / 2, -w / 2), (-h / 2, w / 2), (h / 2, w / 2), (h / 2, -w / 2)]]
        box_rotate = [p.rotate(angle) + position for p in box]
        pygame.draw.polygon(surface, color=color, points=box_rotate)

        # Label
        if label:
            if cls.font is None:
                cls.font = pygame.font.Font(None, 15)
            text = "#{}".format(id(vehicle) % 1000)
            text = cls.font.render(text, 1, (10, 10, 10), (255, 255, 255))
            surface.blit(text, position)

    @classmethod
    def get_color(cls, vehicle) -> Tuple[int]:
        if vehicle.crashed:
            color = cls.RED
        else:
            color = cls.BLUE
        return color


class LaneGraphics:
    """A visualization of a lane."""

    STRIPE_SPACING: float = 5
    """ Offset between stripes [m]"""

    STRIPE_LENGTH: float = 3
    """ Length of a stripe [m]"""

    STRIPE_WIDTH: float = 0.3
    """ Width of a stripe [m]"""

    LANE_LINE_WIDTH: float = 1

    @classmethod
    def display(cls, lane, surface, two_side=True) -> None:
        """
        Display a lane on a surface.

        :param lane: the lane to be displayed
        :param surface: the pygame surface
        :param two_side: draw two sides of the lane, or only one side
        """
        side = 2 if two_side else 1
        stripes_count = int(2 * (surface.get_height() + surface.get_width()) / (cls.STRIPE_SPACING * surface.scaling))
        s_origin, _ = lane.local_coordinates(surface.origin)
        s0 = (int(s_origin) // cls.STRIPE_SPACING - stripes_count // 2) * cls.STRIPE_SPACING
        for side in range(side):
            if lane.line_types[side] == LineType.BROKEN:
                cls.striped_line(lane, surface, stripes_count, s0, side)
            # circular side or continuous, it is same now
            elif lane.line_types[side] == LineType.CONTINUOUS and isinstance(lane, CircularLane):
                cls.continuous_curve(lane, surface, stripes_count, s0, side)
            elif lane.line_types[side] == LineType.SIDE and isinstance(lane, CircularLane):
                cls.continuous_curve(lane, surface, stripes_count, s0, side)
            # the line of continuous straight and side straight is same now
            elif (lane.line_types[side] == LineType.CONTINUOUS) and isinstance(lane, StraightLane):
                cls.continuous_line(lane, surface, stripes_count, s0, side)
            elif (lane.line_types[side] == LineType.SIDE) and isinstance(lane, StraightLane):
                cls.continuous_line(lane, surface, stripes_count, s0, side)
            # special case
            elif lane.line_types[side] == LineType.NONE:
                continue
            else:
                raise ValueError("I don't know how to draw this line type: {}".format(lane.line_types[side]))

    @classmethod
    def striped_line(cls, lane, surface, stripes_count: int, longitudinal: float, side: int) -> None:
        """
        Draw a striped line on one side of a lane, on a surface.

        :param lane: the lane
        :param surface: the pygame surface
        :param stripes_count: the number of stripes to draw
        :param longitudinal: the longitudinal position of the first stripe [m]
        :param side: which side of the road to draw [0:left, 1:right]
        """
        starts = longitudinal + np.arange(stripes_count) * cls.STRIPE_SPACING
        ends = longitudinal + np.arange(stripes_count) * cls.STRIPE_SPACING + cls.STRIPE_LENGTH
        lats = [(side - 0.5) * lane.width_at(s) for s in starts]
        cls.draw_stripes(lane, surface, starts, ends, lats)

    @classmethod
    def continuous_curve(cls, lane, surface, stripes_count: int, longitudinal: float, side: int) -> None:
        """
        Draw a striped line on one side of a lane, on a surface.

        :param lane: the lane
        :param surface: the pygame surface
        :param stripes_count: the number of stripes to draw
        :param longitudinal: the longitudinal position of the first stripe [m]
        :param side: which side of the road to draw [0:left, 1:right]
        """
        starts = longitudinal + np.arange(stripes_count) * cls.STRIPE_SPACING
        ends = longitudinal + np.arange(stripes_count) * cls.STRIPE_SPACING + cls.STRIPE_SPACING
        lats = [(side - 0.5) * lane.width_at(s) for s in starts]
        cls.draw_stripes(lane, surface, starts, ends, lats)

    @classmethod
    def continuous_line(cls, lane, surface, stripes_count: int, longitudinal: float, side: int) -> None:
        """
        Draw a continuous line on one side of a lane, on a surface.

        :param lane: the lane
        :param surface: the pygame surface
        :param stripes_count: the number of stripes that would be drawn if the line was striped
        :param longitudinal: the longitudinal position of the start of the line [m]
        :param side: which side of the road to draw [0:left, 1:right]
        """
        starts = [longitudinal + 0 * cls.STRIPE_SPACING]
        ends = [longitudinal + stripes_count * cls.STRIPE_SPACING + cls.STRIPE_LENGTH]
        lats = [(side - 0.5) * lane.width_at(s) for s in starts]
        cls.draw_stripes(lane, surface, starts, ends, lats)

    @classmethod
    def draw_stripes(cls, lane, surface, starts: List[float], ends: List[float], lats: List[float]) -> None:
        """
        Draw a set of stripes along a lane.

        :param lane: the lane
        :param surface: the surface to draw on
        :param starts: a list of starting longitudinal positions for each stripe [m]
        :param ends: a list of ending longitudinal positions for each stripe [m]
        :param lats: a list of lateral positions for each stripe [m]
        """
        starts = np.clip(starts, 0, lane.length)
        ends = np.clip(ends, 0, lane.length)
        for k, _ in enumerate(starts):
            if abs(starts[k] - ends[k]) > 0.5 * cls.STRIPE_LENGTH:
                pygame.draw.line(
                    surface, surface.WHITE, (surface.vec2pix(lane.position(starts[k], lats[k]))),
                    (surface.vec2pix(lane.position(ends[k], lats[k]))),
                    max(surface.pix(cls.STRIPE_WIDTH), surface.pix(cls.LANE_LINE_WIDTH))
                )

    @classmethod
    def simple_draw(cls, lane, surface, color=(255, 255, 255)):
        from pgdrive.scene_creator.blocks.block import Block
        segment_num = int(lane.length / Block.CIRCULAR_SEGMENT_LENGTH)
        width = lane.width
        for segment in range(segment_num):
            p_1 = lane.position(segment * Block.CIRCULAR_SEGMENT_LENGTH, -width / 2)
            p_2 = lane.position(segment * Block.CIRCULAR_SEGMENT_LENGTH, width / 2)
            p_3 = lane.position((segment + 1) * Block.CIRCULAR_SEGMENT_LENGTH, width / 2)
            p_4 = lane.position((segment + 1) * Block.CIRCULAR_SEGMENT_LENGTH, -width / 2)
            pygame.draw.polygon(
                surface, color,
                [surface.pos2pix(*p_1),
                 surface.pos2pix(*p_2),
                 surface.pos2pix(*p_3),
                 surface.pos2pix(*p_4)]
            )

        # # for last part
        p_1 = lane.position(segment_num * Block.CIRCULAR_SEGMENT_LENGTH, -width / 2)
        p_2 = lane.position(segment_num * Block.CIRCULAR_SEGMENT_LENGTH, width / 2)
        p_3 = lane.position(lane.length, width / 2)
        p_4 = lane.position(lane.length, -width / 2)
        pygame.draw.polygon(
            surface, color,
            [surface.pos2pix(*p_1),
             surface.pos2pix(*p_2),
             surface.pos2pix(*p_3),
             surface.pos2pix(*p_4)]
        )


class ObservationWindowMultiChannel:
    CHANNEL_NAMES = ["road_network", "traffic_flow", "target_vehicle", "navigation", "past_pos"]

    def __init__(self, names, max_range, resolution):
        assert isinstance(names, list)
        assert set(self.CHANNEL_NAMES)
        self.sub_observations = {
            k: ObservationWindow(max_range=max_range, resolution=resolution)
            for k in ["traffic_flow", "target_vehicle", "navigation"]
        }
        self.sub_observations["road_network"] = ObservationWindow(
            max_range=max_range, resolution=(resolution[0] * 2, resolution[1] * 2)
        )

    def reset(self, canvas_runtime):
        for k, sub in self.sub_observations.items():
            sub.reset(canvas_runtime)

    def render(self, canvas_dict, position, heading):
        assert isinstance(canvas_dict, dict)
        assert set(canvas_dict.keys()) == set(self.sub_observations.keys())
        ret = dict()
        for k, canvas in canvas_dict.items():
            ret[k] = self.sub_observations[k].render(canvas, position, heading)
        return self.get_observation_window(ret)

    def get_observation_window(self, canvas_dict=None):
        if canvas_dict is None:
            canvas_dict = {k: v.get_observation_window() for k, v in self.sub_observations.items()}
        return canvas_dict

    def get_size(self):
        return next(iter(self.sub_observations.values())).get_size()
