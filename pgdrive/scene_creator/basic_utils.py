import copy
from typing import List, TYPE_CHECKING

from pgdrive.scene_creator.lanes.lane import AbstractLane, LineType

if TYPE_CHECKING:
    from pgdrive.scene_creator.road.road_network import RoadNetwork
    from pgdrive.scene_creator.road.road import Road
    from pgdrive.scene_creator.blocks.block import BlockSocket
    from pgdrive.scene_creator.lanes.straight_lane import StraightLane


class Decoration:
    start = "decoration"
    end = "decoration_"


class Goal:
    """
    Goal at intersection
    The keywords 0, 1, 2 should be reserved, and only be used in roundabout and intersection
    """

    RIGHT = 0
    STRAIGHT = 1
    LEFT = 2
    ADVERSE = 3  # Useless now


def get_lanes_on_road(road: "Road", roadnet: "RoadNetwork") -> List[AbstractLane]:
    return roadnet.graph[road.start_node][road.end_node]


def block_socket_merge(
    socket_1: "BlockSocket", socket_2: "BlockSocket", global_network: "RoadNetwork", positive_merge: False
):
    global_network.graph[socket_1.positive_road.start_node][socket_2.negative_road.start_node] = \
        global_network.graph[socket_1.positive_road.start_node].pop(socket_1.positive_road.end_node)

    global_network.graph[socket_2.positive_road.start_node][socket_1.negative_road.start_node] = \
        global_network.graph[socket_2.positive_road.start_node].pop(socket_2.positive_road.end_node)
