import math
from pgdrive.constants import CollisionGroup
from pgdrive.engine.physics_node import BaseRigidBodyNode
from typing import List, TYPE_CHECKING, Tuple, Union

import numpy as np
from panda3d.bullet import BulletBoxShape, BulletCylinderShape, ZUp
from panda3d.core import TransformState, NodePath
from panda3d.core import Vec3

from pgdrive.component.lane.abs_lane import AbstractLane
from pgdrive.component.lane.circular_lane import CircularLane
from pgdrive.constants import Decoration, BodyName
from pgdrive.engine.core.engine_core import EngineCore
from pgdrive.utils.coordinates_shift import panda_heading
from pgdrive.utils.coordinates_shift import panda_position
from pgdrive.utils.math_utils import get_points_bounding_box, norm
from pgdrive.utils.utils import get_object_from_node

if TYPE_CHECKING:
    from pgdrive.component.blocks.pg_block import PGBlockSocket
    from pgdrive.component.road.road import Road
    from pgdrive.component.road.road_network import RoadNetwork


def get_lanes_on_road(road: "Road", roadnet: "RoadNetwork") -> List["AbstractLane"]:
    return roadnet.graph[road.start_node][road.end_node]


def block_socket_merge(
        socket_1: "PGBlockSocket", socket_2: "PGBlockSocket", global_network: "RoadNetwork", positive_merge: False
):
    global_network.graph[socket_1.positive_road.start_node][socket_2.negative_road.start_node] = \
        global_network.graph[socket_1.positive_road.start_node].pop(socket_1.positive_road.end_node)

    global_network.graph[socket_2.positive_road.start_node][socket_1.negative_road.start_node] = \
        global_network.graph[socket_2.positive_road.start_node].pop(socket_2.positive_road.end_node)


def check_lane_on_road(road_network: "RoadNetwork", lane, positive: float = 0, ignored=None) -> bool:
    """
    Calculate if the new lane intersects with other lanes in current road network
    The return Value is True when cross
    Note: the decoration road will be ignored in default
    """
    graph = road_network.graph
    for _from, to_dict in graph.items():
        for _to, lanes in to_dict.items():
            if ignored and (_from, _to) == ignored:
                continue
            if (_from, _to) == (Decoration.start, Decoration.end):
                continue
            if len(lanes) == 0:
                continue
            x_max_1, x_min_1, y_max_1, y_min_1 = get_road_bounding_box(lanes)
            x_max_2, x_min_2, y_max_2, y_min_2 = get_road_bounding_box([lane])
            if x_min_1 > x_max_2 or x_min_2 > x_max_1 or y_min_1 > y_max_2 or y_min_2 > y_max_1:
                continue
            for _id, l in enumerate(lanes):
                for i in range(1, int(lane.length), 1):
                    sample_point = lane.position(i, positive * lane.width_at(i) / 2.0)
                    longitudinal, lateral = l.local_coordinates(sample_point)
                    is_on = math.fabs(lateral) <= l.width_at(longitudinal) / 2.0 and 0 <= longitudinal <= l.length
                    if is_on:
                        return True
    return False


def get_road_bounding_box(lanes, extra_lateral=3) -> Tuple:
    """
    Return (x_max, x_min, y_max, y_min) as bounding box of this road
    :param lanes: Lanes in this road
    :param extra_lateral: extra width in lateral direction, usually sidewalk width
    :return: x_max, x_min, y_max, y_min
    """
    line_points = get_curve_contour(lanes, extra_lateral) if isinstance(lanes[0], CircularLane) \
        else get_straight_contour(lanes, extra_lateral)
    return get_points_bounding_box(line_points)


def get_straight_contour(lanes, extra_lateral) -> List:
    """
    Get several points as bounding box of this road
    :param lanes: lanes contained in road
    :param extra_lateral: extra length in lateral direction, usually sidewalk
    :return: points
    :param lanes:
    :return:
    """
    ret = []
    for lane, dir in [(lanes[0], -1), (lanes[-1], 1)]:
        ret.append(lane.position(0.1, dir * (lane.width / 2.0 + extra_lateral)))
        ret.append(lane.position(lane.length - 0.1, dir * (lane.width / 2.0 + extra_lateral)))
    return ret


def get_curve_contour(lanes, extra_lateral) -> List:
    """
    Get several points as bounding box of this road
    :param lanes: lanes contained in road
    :param extra_lateral: extra length in lateral direction, usually sidewalk
    :return: points
    """
    points = []
    for lane, lateral_dir in [(lanes[0], -1), (lanes[-1], 1)]:
        pi_2 = (np.pi / 2.0)
        points += [
            lane.position(0.1, lateral_dir * (lane.width / 2.0 + extra_lateral)),
            lane.position(lane.length - 0.1, lateral_dir * (lane.width / 2.0 + extra_lateral))
        ]
        start_phase = (lane.start_phase // pi_2) * pi_2
        start_phase += pi_2 if lane.direction == 1 else 0
        for phi_index in range(4):
            phi = start_phase + phi_index * pi_2 * lane.direction
            if lane.direction * phi > lane.direction * lane.end_phase:
                break
            point = lane.center + (lane.radius - lateral_dir * (lane.width / 2.0 + extra_lateral) *
                                   lane.direction) * np.array([math.cos(phi), math.sin(phi)])
            points.append(point)
    return points


def get_all_lanes(roadnet: "RoadNetwork"):
    graph = roadnet.graph
    res = []
    for from_, to_dict in graph.items():
        for _to, lanes in to_dict.items():
            for l in lanes:
                res.append(l)
    return res


def ray_localization(heading: tuple,
                     position: tuple,
                     engine: EngineCore,
                     return_all_result=False) -> Union[List[Tuple], Tuple]:
    """
    Get the index of the lane closest to a physx_world position.
    Only used when smoething is on lane ! Otherwise fall back to use get_closest_lane()
    :param heading: heading to help filter lanes
    :param position: a physx_world position [m].
    :param engine: PGWorld class
    :param return_all_result: return a list instead of the lane with min L1 distance
    :return: list(closest lane) or closest lane.
    """
    results = engine.physics_world.static_world.rayTestAll(
        panda_position(position, 1.0), panda_position(position, -1.0)
    )
    lane_index_dist = []
    if results.hasHits():
        for res in results.getHits():
            if res.getNode().getName() == BodyName.Lane:
                lane = get_object_from_node(res.getNode())
                long, _ = lane.local_coordinates(position)
                lane_heading = lane.heading_at(long)

                # dir = np.array([math.cos(lane_heading), math.sin(lane_heading)])
                # dot_result = dir.dot(heading)

                dot_result = math.cos(lane_heading) * heading[0] + math.sin(lane_heading) * heading[1]
                cosangle = dot_result / (
                        norm(math.cos(lane_heading), math.sin(lane_heading)) * norm(heading[0], heading[1])
                )

                if cosangle > 0:
                    lane_index_dist.append((lane, lane.index, lane.distance(position)))
    if return_all_result:
        ret = []
        if len(lane_index_dist) > 0:
            for lane, index, dist in lane_index_dist:
                ret.append((lane, index, dist))
        sorted(ret, key=lambda k: k[2])
        return ret
    else:
        if len(lane_index_dist) > 0:
            ret_index = np.argmin([d for _, _, d in lane_index_dist])
            lane, index, dist = lane_index_dist[ret_index]
        else:
            lane, index, dist = None, None, None
        return lane, index


def rect_region_detection(
        engine: EngineCore,
        position: Tuple,
        heading: float,
        heading_direction_length: float,
        side_direction_width: float,
        detection_group: int,
        height=10,
        in_static_world=False
):
    """

     ----------------------------------
     |               *                |  --->>>
     ----------------------------------
     * position
     --->>> heading direction
     ------ longitude length
     | lateral width

     **CAUTION**: position is the middle point of longitude edge

    :param engine: BaseEngine class
    :param position: position in PGDrive
    :param heading: heading in PGDrive [degree]
    :param heading_direction_length: rect length in heading direction
    :param side_direction_width: rect width in side direction
    :param detection_group: which group to detect
    :param height: the detect will be executed from this height to 0
    :param in_static_world: execute detection in static world
    :return: detection result
    """
    region_detect_start = panda_position(position, z=height)
    region_detect_end = panda_position(position, z=-1)
    tsFrom = TransformState.makePosHpr(region_detect_start, Vec3(panda_heading(heading), 0, 0))
    tsTo = TransformState.makePosHpr(region_detect_end, Vec3(panda_heading(heading), 0, 0))

    shape = BulletBoxShape(Vec3(heading_direction_length / 2, side_direction_width / 2, 1))
    penetration = 0.0

    physics_world = engine.physics_world.dynamic_world if not in_static_world else engine.physics_world.static_world

    result = physics_world.sweep_test_closest(shape, tsFrom, tsTo, detection_group, penetration)
    return result


def circle_region_detection(
        engine: EngineCore, position: Tuple, radius: float, detection_group: int, height=10, in_static_world=False
):
    """
    :param engine: BaseEngine class
    :param position: position in PGDrive
    :param radius: radius of the region to be detected
    :param detection_group: which group to detect
    :param height: the detect will be executed from this height to 0
    :param in_static_world: execute detection in static world
    :return: detection result
    """
    region_detect_start = panda_position(position, z=height)
    region_detect_end = panda_position(position, z=-1)
    tsFrom = TransformState.makePos(region_detect_start)
    tsTo = TransformState.makePos(region_detect_end)

    shape = BulletCylinderShape(radius, 5, ZUp)
    penetration = 0.0

    physics_world = engine.physics_world.dynamic_world if not in_static_world else engine.physics_world.static_world

    result = physics_world.sweep_test_closest(shape, tsFrom, tsTo, detection_group, penetration)
    return result


def generate_invisible_static_wall(
        heading_length: float,
        side_width: float,
        height=10,
        object_id=None,
        type_name=BodyName.InvisibleWall,
        collision_group=CollisionGroup.InvisibleWall
):
    """
    Add an invisible physics wall to physics world
    You can add some building models to the same location, add make it be detected by lidar
    ----------------------------------
    |               *                |  --->>>
    ----------------------------------
    * position
    --->>> heading direction
    ------ longitude length
    | lateral width

    **CAUTION**: position is the middle point of longitude edge
    :param heading_length: rect length in heading direction
    :param side_width: rect width in side direction
    :param height: the detect will be executed from this height to 0
    :param object_id: name of this invisible wall
    :param type_name: default invisible wall
    :param collision_group: control the collision of this static wall and other elements
    :return node_path
    """
    shape = BulletBoxShape(Vec3(heading_length / 2, side_width / 2, height))
    body_node = BaseRigidBodyNode(object_id, type_name)
    body_node.setActive(False)
    body_node.setKinematic(False)
    body_node.setStatic(True)
    body_node.addShape(shape)
    body_node.setIntoCollideMask(collision_group)
    return body_node


def is_same_lane_index(lane_index_1, lane_index_2):
    return all([lane_index_1[i] == lane_index_2[i] for i in range(3)])


def is_following_lane_index(current_lane_index, next_lane_index):
    return True if current_lane_index[1] == next_lane_index[0] and current_lane_index[-1] == next_lane_index[
        -1] else False
