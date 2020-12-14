import numpy as np
import pygame
from typing import List, Tuple
from pg_drive.scene_creator.lanes.lane import LineType
from pg_drive.scene_creator.lanes.straight_lane import StraightLane


class HighwayRender:
    """
    Most of the source code is from Highway-Env, we only optimize and integrate it in PG-Drive
    See more information on its Github page: https://github.com/eleurent/highway-env
    """

    def __init__(self):
        self.map_sufface = None

    def draw_scene(self) -> np.ndarray:
        pass


class VehicleGraphics(object):
    RED = (255, 100, 100)
    GREEN = (50, 200, 0)
    BLUE = (100, 200, 255)
    YELLOW = (200, 200, 0)
    BLACK = (60, 60, 60)
    PURPLE = (200, 0, 150)
    DEFAULT_COLOR = YELLOW
    EGO_COLOR = GREEN

    @classmethod
    def display(cls, vehicle, surface, transparent: bool = False,
                offscreen: bool = False,
                label: bool = False) -> None:
        """
        Display a vehicle on a pygame surface.

        The vehicle is represented as a colored rotated rectangle.

        :param vehicle: the vehicle to be drawn
        :param surface: the surface to draw the vehicle on
        :param transparent: whether the vehicle should be drawn slightly transparent
        :param offscreen: whether the rendering should be done offscreen or not
        :param label: whether a text label should be rendered
        """
        if not surface.is_visible(vehicle.position):
            return

        v = vehicle
        tire_length, tire_width = 1, 0.3

        # Vehicle rectangle
        length = v.LENGTH + 2 * tire_length
        vehicle_surface = pygame.Surface((surface.pix(length), surface.pix(length)),
                                         flags=pygame.SRCALPHA)  # per-pixel alpha
        rect = (surface.pix(tire_length), surface.pix(length / 2 - v.WIDTH / 2), surface.pix(v.LENGTH),
                surface.pix(v.WIDTH))
        pygame.draw.rect(vehicle_surface, cls.get_color(v, transparent), rect, 0)
        pygame.draw.rect(vehicle_surface, cls.BLACK, rect, 1)

        # # Tires
        # if type(vehicle) in [Vehicle, BicycleVehicle]:
        #     tire_positions = [[surface.pix(tire_length), surface.pix(length / 2 - v.WIDTH / 2)],
        #                       [surface.pix(tire_length), surface.pix(length / 2 + v.WIDTH / 2)],
        #                       [surface.pix(length - tire_length), surface.pix(length / 2 - v.WIDTH / 2)],
        #                       [surface.pix(length - tire_length), surface.pix(length / 2 + v.WIDTH / 2)]]
        #     tire_angles = [0, 0, v.action["steering"], v.action["steering"]]
        #     for tire_position, tire_angle in zip(tire_positions, tire_angles):
        #         tire_surface = pygame.Surface((surface.pix(tire_length), surface.pix(tire_length)),
        #                                       pygame.SRCALPHA)
        #         rect = (0, surface.pix(tire_length / 2 - tire_width / 2), surface.pix(tire_length),
        #                 surface.pix(tire_width))
        #         pygame.draw.rect(tire_surface, cls.BLACK, rect, 0)
        #         cls.blit_rotate(vehicle_surface, tire_surface, tire_position, np.rad2deg(-tire_angle))

        # Centered rotation
        h = v.heading if abs(v.heading) > 2 * np.pi / 180 else 0
        position = [*surface.pos2pix(v.position[0], v.position[1])]
        if not offscreen:
            # convert_alpha throws errors in offscreen mode
            # see https://stackoverflow.com/a/19057853
            vehicle_surface = pygame.Surface.convert_alpha(vehicle_surface)
        cls.blit_rotate(surface, vehicle_surface, position, np.rad2deg(-h))

        # Label
        if label:
            font = pygame.font.Font(None, 15)
            text = "#{}".format(id(v) % 1000)
            text = font.render(text, 1, (10, 10, 10), (255, 255, 255))
            surface.blit(text, position)

    @staticmethod
    def blit_rotate(surf: pygame.SurfaceType, image: pygame.SurfaceType, pos, angle: float,
                    origin_pos=None, show_rect: bool = False) -> None:
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
        origin = (pos[0] - origin_pos[0] + min_box[0] - pivot_move[0],
                  pos[1] - origin_pos[1] - max_box[1] + pivot_move[1])
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


class LaneGraphics(object):
    """A visualization of a lane."""

    STRIPE_SPACING: float = 5
    """ Offset between stripes [m]"""

    STRIPE_LENGTH: float = 3
    """ Length of a stripe [m]"""

    STRIPE_WIDTH: float = 0.3
    """ Width of a stripe [m]"""

    @classmethod
    def display(cls, lane, surface) -> None:
        """
        Display a lane on a surface.

        :param lane: the lane to be displayed
        :param surface: the pygame surface
        """
        stripes_count = int(
            2 * (surface.get_height() + surface.get_width()) / (cls.STRIPE_SPACING * surface.scaling))
        s_origin, _ = lane.local_coordinates(surface.origin)
        s0 = (int(s_origin) // cls.STRIPE_SPACING - stripes_count // 2) * cls.STRIPE_SPACING
        for side in range(2):
            if lane.line_types[side] == LineType.STRIPED:
                cls.striped_line(lane, surface, stripes_count, s0, side)
            elif lane.line_types[side] == LineType.CONTINUOUS:
                cls.continuous_curve(lane, surface, stripes_count, s0, side)
            elif lane.line_types[side] == LineType.CONTINUOUS and isinstance(lane, StraightLane):
                cls.continuous_line(lane, surface, stripes_count, s0, side)

    @classmethod
    def striped_line(cls, lane, surface, stripes_count: int, longitudinal: float,
                     side: int) -> None:
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
    def continuous_curve(cls, lane, surface, stripes_count: int,
                         longitudinal: float, side: int) -> None:
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
    def continuous_line(cls, lane, surface, stripes_count: int, longitudinal: float,
                        side: int) -> None:
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
    def draw_stripes(cls, lane, surface,
                     starts: List[float], ends: List[float], lats: List[float]) -> None:
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
                pygame.draw.line(surface, surface.WHITE,
                                 (surface.vec2pix(lane.position(starts[k], lats[k]))),
                                 (surface.vec2pix(lane.position(ends[k], lats[k]))),
                                 max(surface.pix(cls.STRIPE_WIDTH), 1))

    @classmethod
    def draw_ground(cls, lane, surface, color: Tuple[float], width: float,
                    draw_surface: pygame.Surface = None) -> None:
        draw_surface = draw_surface or surface
        stripes_count = int(
            2 * (surface.get_height() + surface.get_width()) / (cls.STRIPE_SPACING * surface.scaling))
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


class RoadGraphics(object):
    """A visualization of a road lanes."""

    @staticmethod
    def display(road, surface):
        """
        Display the road lanes on a surface.

        :param road: the road to be displayed
        :param surface: the pygame surface
        """
        surface.fill(surface.GREY)
        for _from in road.network.graph.keys():
            for _to in road.network.graph[_from].keys():
                for l in road.network.graph[_from][_to]:
                    LaneGraphics.display(l, surface)

    @staticmethod
    def display_traffic(road, surface, simulation_frequency: int = 15,
                        offscreen: bool = False) \
            -> None:
        """
        Display the road vehicles on a surface.

        :param road: the road to be displayed
        :param surface: the pygame surface
        :param simulation_frequency: simulation frequency
        :param offscreen: render without displaying on a screen
        """
        if road.record_history:
            for v in road.vehicles:
                VehicleGraphics.display_history(v, surface, simulation=simulation_frequency,
                                                offscreen=offscreen)
        for v in road.vehicles:
            VehicleGraphics.display(v, surface, offscreen=offscreen)
