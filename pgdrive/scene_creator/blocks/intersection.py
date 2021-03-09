import copy
from collections import deque

import numpy as np

from pgdrive.pg_config.parameter_space import Parameter, BlockParameterSpace
from pgdrive.pg_config.pg_space import PGSpace
from pgdrive.scene_creator.blocks.block import Block, BlockSocket
from pgdrive.scene_creator.blocks.create_block_utils import CreateAdverseRoad, CreateRoadFrom, ExtendStraightLane, \
    sharpbend
from pgdrive.scene_creator.lane.abs_lane import LineType
from pgdrive.scene_creator.lane.straight_lane import StraightLane
from pgdrive.scene_creator.road.road import Road
from pgdrive.utils.scene_utils import check_lane_on_road


class InterSection(Block):
    """
                                up(Goal:1)
                                   ||
                                   ||
                                   ||
                                   ||
                                   ||
                  _________________||_________________
    left(Goal:2)  -----------------||---------------- right(Goal:0)
                               __  ||
                              |    ||
             spawn_road    <--|    ||
                              |    ||
                              |__  ||
                                  down
    It's an Intersection with two lanes on same direction, 4 lanes on both roads
    """

    ID = "X"
    EXTRA_PART = "extra"
    PARAMETER_SPACE = PGSpace(BlockParameterSpace.INTERSECTION)
    SOCKET_NUM = 3
    ANGLE = 90  # may support other angle in the future
    EXIT_PART_LENGTH = 30

    # LEFT_TURN_NUM = 1 now it is useless

    def _try_plug_into_previous_block(self) -> bool:
        para = self.get_config()
        decrease_increase = -1 if para[Parameter.decrease_increase] == 0 else 1
        if self.positive_lane_num <= 1:
            decrease_increase = 1
        elif self.positive_lane_num >= 4:
            decrease_increase = -1
        self.lane_num_intersect = self.positive_lane_num + decrease_increase * para[Parameter.change_lane_num]
        no_cross = True
        attach_road = self.pre_block_socket.positive_road
        _attach_road = self.pre_block_socket.negative_road
        attach_lanes = attach_road.get_lanes(self._global_network)
        # right straight left node name, rotate it to fit different part
        intersect_nodes = deque(
            [self.road_node(0, 0),
             self.road_node(1, 0),
             self.road_node(2, 0), _attach_road.start_node]
        )

        for i in range(4):
            right_lane, success = self._create_part(
                attach_lanes, attach_road, para[Parameter.radius], intersect_nodes, i
            )
            no_cross = no_cross and success
            if i != 3:
                lane_num = self.positive_lane_num if i == 1 else self.lane_num_intersect
                exit_road = Road(self.road_node(i, 0), self.road_node(i, 1))
                no_cross = CreateRoadFrom(
                    right_lane, lane_num, exit_road, self.block_network, self._global_network
                ) and no_cross
                no_cross = CreateAdverseRoad(exit_road, self.block_network, self._global_network) and no_cross
                socket = BlockSocket(exit_road, -exit_road)
                self.add_reborn_roads(socket.negative_road)
                self.add_sockets(socket)
                attach_road = -exit_road
                attach_lanes = attach_road.get_lanes(self.block_network)
        return no_cross

    def _create_part(self, attach_lanes, attach_road: Road, radius: float, intersect_nodes: deque,
                     part_idx) -> (StraightLane, bool):
        lane_num = self.lane_num_intersect if part_idx == 0 or part_idx == 2 else self.positive_lane_num
        non_cross = True
        attach_left_lane = attach_lanes[0]
        # first left part
        assert isinstance(attach_left_lane, StraightLane), "Can't create a intersection following a circular lane"
        self._create_left_turn(radius, lane_num, attach_left_lane, attach_road, intersect_nodes, part_idx)

        # go forward part
        lanes_on_road = copy.deepcopy(attach_lanes)
        straight_lane_len = 2 * radius + (2 * lane_num - 1) * lanes_on_road[0].width_at(0)
        for l in lanes_on_road:
            next_lane = ExtendStraightLane(l, straight_lane_len, (LineType.NONE, LineType.NONE))
            self.block_network.add_lane(attach_road.end_node, intersect_nodes[1], next_lane)

        # right part
        length = self.EXIT_PART_LENGTH
        right_turn_lane = lanes_on_road[-1]
        assert isinstance(right_turn_lane, StraightLane), "Can't create a intersection following a circular lane"
        right_bend, right_straight = sharpbend(
            right_turn_lane, length, radius, np.deg2rad(self.ANGLE), True, right_turn_lane.width_at(0),
            (LineType.NONE, LineType.SIDE)
        )

        non_cross = (not check_lane_on_road(self._global_network, right_bend, 1)) and non_cross
        CreateRoadFrom(
            right_bend,
            min(self.positive_lane_num, self.lane_num_intersect),
            Road(attach_road.end_node, intersect_nodes[0]),
            self.block_network,
            self._global_network,
            toward_smaller_lane_index=True,
            side_lane_line_type=LineType.SIDE,
            inner_lane_line_type=LineType.NONE,
            center_line_type=LineType.NONE
        )

        intersect_nodes.rotate(-1)
        right_straight.line_types = [LineType.BROKEN, LineType.SIDE]
        return right_straight, non_cross

    def get_socket(self, index: int) -> BlockSocket:
        socket = super(InterSection, self).get_socket(index)
        self._reborn_roads.remove(socket.negative_road)
        return socket

    def _create_left_turn(self, radius, lane_num, attach_left_lane, attach_road, intersect_nodes, part_idx):
        left_turn_radius = radius + lane_num * attach_left_lane.width_at(0)
        diff = self.lane_num_intersect - self.positive_lane_num  # increase lane num
        if ((part_idx == 1 or part_idx == 3) and diff > 0) or ((part_idx == 0 or part_idx == 2) and diff < 0):
            diff = abs(diff)
            left_bend, extra_part = sharpbend(
                attach_left_lane, self.lane_width * diff, left_turn_radius, np.deg2rad(self.ANGLE), False,
                attach_left_lane.width_at(0), (LineType.NONE, LineType.NONE)
            )
            left_road_start = intersect_nodes[2]
            pre_left_road_start = left_road_start + self.EXTRA_PART
            CreateRoadFrom(
                left_bend,
                min(self.positive_lane_num, self.lane_num_intersect),
                Road(attach_road.end_node, pre_left_road_start),
                self.block_network,
                self._global_network,
                toward_smaller_lane_index=False,
                center_line_type=LineType.NONE,
                side_lane_line_type=LineType.NONE,
                inner_lane_line_type=LineType.NONE
            )

            CreateRoadFrom(
                extra_part,
                min(self.positive_lane_num, self.lane_num_intersect),
                Road(pre_left_road_start, left_road_start),
                self.block_network,
                self._global_network,
                toward_smaller_lane_index=False,
                center_line_type=LineType.NONE,
                side_lane_line_type=LineType.NONE,
                inner_lane_line_type=LineType.NONE
            )

        else:
            left_bend, _ = sharpbend(
                attach_left_lane, self.EXIT_PART_LENGTH, left_turn_radius, np.deg2rad(self.ANGLE), False,
                attach_left_lane.width_at(0), (LineType.NONE, LineType.NONE)
            )
            left_road_start = intersect_nodes[2]
            CreateRoadFrom(
                left_bend,
                min(self.positive_lane_num, self.lane_num_intersect),
                Road(attach_road.end_node, left_road_start),
                self.block_network,
                self._global_network,
                toward_smaller_lane_index=False,
                center_line_type=LineType.NONE,
                side_lane_line_type=LineType.NONE,
                inner_lane_line_type=LineType.NONE
            )
