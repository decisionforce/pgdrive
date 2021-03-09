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
        self.canvas_unscaled = None
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
        self.canvas_unscaled = pygame.Surface(self.receptive_field)
        self.canvas_unscaled.fill(COLOR_BLACK)
        self.canvas_display.fill(COLOR_BLACK)

    def render(self, canvas, position, heading):
        # Prepare a runtime canvas for rotation
        # Assume max_range is only the radius!
        self.canvas_rotate.blit(
            canvas, (0, 0), (
                position[0] - self.receptive_field_double[0] / 2, position[1] - self.receptive_field_double[1] / 2,
                self.receptive_field_double[0], self.receptive_field_double[1]
            )
        )

        # Rotate the image so that ego is always heading top
        rotation = np.rad2deg(heading) + 90
        new_canvas = pygame.transform.rotate(self.canvas_rotate, rotation)
        # new_canvas = pygame.transform.rotozoom(self.canvas_rotate, rotation, 1)  # Optional choice! Not so efficient!

        # Crop the rotated image and then resize to the desired resolution
        self.canvas_unscaled.blit(
            new_canvas,
            (0, 0),
            (
                new_canvas.get_size()[0] / 2 - self.receptive_field[0] / 2,  # Left
                new_canvas.get_size()[1] / 2 - self.receptive_field[1] / 2,  # Top
                self.receptive_field[0],  # Width
                self.receptive_field[1]  # Height
            )
        )
        pygame.transform.smoothscale(self.canvas_unscaled, self.canvas_display.get_size(), self.canvas_display)

        # print(
        #     "Current canvas size, canvas display size",
        #     new_canvas.get_size(),
        #     canvas.get_size(),
        #     self.receptive_field,
        #     self.receptive_field_double,
        #     canvas.get_size(),
        #     self.canvas_display.get_size()
        # )
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

    # registered_surface = dict()

    @classmethod
    def get_surface(cls, length):
        # print(cls.registered_surface.keys())
        # if length in cls.registered_surface:
        #     return cls.registered_surface[length]
        vehicle_surface = pygame.Surface(
            (length, length), flags=pygame.SRCALPHA
        )  # per-pixel alpha
        # cls.registered_surface[length] = vehicle_surface
        return vehicle_surface

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

        v = vehicle
        tire_length, tire_width = 1, 0.3

        # Vehicle rectangle
        length = v.LENGTH + 2 * tire_length

        vehicle_surface = cls.get_surface(surface.pix(length))
        rect = (
            surface.pix(tire_length), surface.pix(length / 2 - v.WIDTH / 2), surface.pix(v.LENGTH),
            surface.pix(v.WIDTH)
        )
        pygame.draw.rect(vehicle_surface, color, rect, 0)

        # Old Highway heritage, draw black curve as the contour of vehicle.
        # pygame.draw.rect(vehicle_surface, cls.BLACK, rect, 1)

        # Centered rotation
        position = [*surface.pos2pix(v.position[0], v.position[1])]

        cls.blit_rotate(surface, vehicle_surface, position, np.rad2deg(-heading))

        # Label
        if label:
            if cls.font is None:
                cls.font = pygame.font.Font(None, 15)
            text = "#{}".format(id(v) % 1000)
            text = cls.font.render(text, 1, (10, 10, 10), (255, 255, 255))
            surface.blit(text, position)

    @staticmethod
    def blit_rotate(
            surf: pygame.SurfaceType,
            image: pygame.SurfaceType,
            pos,
            angle: float,
            origin_pos=None,
            show_rect: bool = False
    ) -> None:
        """Many thanks to https://stackoverflow.com/a/54714144."""
        # calculate the axis aligned bounding box of the rotated image
        w, h = image.get_size()
        box = [pygame.math.Vector2(p) for p in [(0, 0), (w, 0), (w, -h), (0, -h)]]
        box_rotate = [p.rotate(angle) for p in box]
        min_box = (min(box_rotate, key=lambda p: p[0])[0], min(box_rotate, key=lambda p: p[1])[1])
        max_box = (max(box_rotate, key=lambda p: p[0])[0], max(box_rotate, key=lambda p: p[1])[1])

        # calculate the translation of the pivot
        if origin_pos is None:
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
        # draw rectangle around the image
        if show_rect:
            pygame.draw.rect(surf, (255, 0, 0), (*origin, *rotated_image.get_size()), 2)

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

    @classmethod
    def draw_ground(cls, lane, surface, color: Tuple[float], width: float, draw_surface: pygame.Surface = None) -> None:
        draw_surface = draw_surface or surface
        stripes_count = int(2 * (surface.get_height() + surface.get_width()) / (cls.STRIPE_SPACING * surface.scaling))
        s_origin, _ = lane.local_coordinates(surface.origin)
        s0 = (int(s_origin) // cls.STRIPE_SPACING - stripes_count // 2) * cls.STRIPE_SPACING
        dots = []
        for side in range(2):
            longis = np.clip(s0 + np.arange(stripes_count) * cls.STRIPE_SPACING, 0, lane.length)
            lats = [2 * (side - 0.5) * width for _ in longis]
            new_dots = [surface.vec2pix(lane.position(longi, lat)) for longi, lat in zip(longis, lats)]
            new_dots = reversed(new_dots) if side else new_dots
            dots.extend(new_dots)
        pygame.draw.polygon(draw_surface, color, dots, 0)


class ObservationWindowMultiChannel:
    def __init__(self, names, max_range, resolution):
        assert isinstance(names, list)
        self.sub_observations = {
            k: ObservationWindow(max_range=max_range, resolution=resolution) for k in names
        }

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
