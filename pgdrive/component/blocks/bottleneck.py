from pgdrive.component.blocks.create_block_utils import CreateAdverseRoad, CreateRoadFrom, ExtendStraightLane, \
    create_wave_lanes
from pgdrive.component.blocks.pg_block import PGBlock, PGBlockSocket
from pgdrive.component.road.road import Road
from pgdrive.constants import LineType
from pgdrive.utils.space import ParameterSpace, Parameter, BlockParameterSpace


class Bottleneck(PGBlock):
    """
    This block is used to change thr lane num
    """
    ID = None
    SOCKET_NUM = 1
    PARAMETER_SPACE = ParameterSpace(BlockParameterSpace.BOTTLENECK_PARAMETER)

    # property of bottleneck
    BOTTLENECK_LEN = None


class Merge(Bottleneck):
    """
    -----\
          \
           -------------------
           -------------------
          /
    -----/
    InBottlecneck
    """
    ID = "y"

    def _try_plug_into_previous_block(self) -> bool:
        no_cross = True
        parameters = self.get_config()
        self.BOTTLENECK_LEN = parameters["bottle_len"]
        lane_num_changed = parameters[Parameter.lane_num]

        start_ndoe = self.pre_block_socket.positive_road.end_node
        straight_lane_num = int(self.positive_lane_num - lane_num_changed)
        straight_lane_num = max(1, straight_lane_num)

        circular_lane_num = self.positive_lane_num - straight_lane_num

        # part 1, straight road 0
        basic_lane = self.positive_lanes[straight_lane_num - 1]
        ref_lane = ExtendStraightLane(basic_lane, self.BOTTLENECK_LEN, [LineType.NONE, LineType.NONE])
        straight_road = Road(start_ndoe, self.road_node(0, 0))
        no_cross = CreateRoadFrom(
            ref_lane,
            straight_lane_num,
            straight_road,
            self.block_network,
            self._global_network,
            center_line_type=LineType.CONTINUOUS,
            side_lane_line_type=LineType.NONE,
            inner_lane_line_type=LineType.NONE
        ) and no_cross
        no_cross = CreateAdverseRoad(
            straight_road,
            self.block_network,
            self._global_network,
            inner_lane_line_type=LineType.NONE,
            side_lane_line_type=LineType.NONE,
            center_line_type=LineType.CONTINUOUS
        ) and no_cross

        # extend for socket ,part 1 road 1
        ref_lane = ExtendStraightLane(ref_lane, parameters[Parameter.length], [LineType.NONE, LineType.NONE])
        socket_road = Road(self.road_node(0, 0), self.road_node(0, 1))
        no_cross = CreateRoadFrom(
            ref_lane,
            straight_lane_num,
            socket_road,
            self.block_network,
            self._global_network,
            center_line_type=LineType.CONTINUOUS,
            side_lane_line_type=LineType.SIDE,
            inner_lane_line_type=LineType.BROKEN
        ) and no_cross
        no_cross = CreateAdverseRoad(
            socket_road,
            self.block_network,
            self._global_network,
            inner_lane_line_type=LineType.BROKEN,
            side_lane_line_type=LineType.SIDE,
            center_line_type=LineType.CONTINUOUS
        ) and no_cross

        negative_sockect_road = -socket_road
        self.add_sockets(PGBlockSocket(socket_road, negative_sockect_road))

        # part 2, circular part
        for index, lane in enumerate(self.positive_lanes[straight_lane_num:], 1):
            lateral_dist = index * self.lane_width / 2
            inner_node = self.road_node(1, index)
            side_line_type = LineType.SIDE if index == self.positive_lane_num - straight_lane_num else LineType.NONE

            # positive part
            circular_1, circular_2, _ = create_wave_lanes(lane, lateral_dist, self.BOTTLENECK_LEN, 5, self.lane_width)
            road_1 = Road(start_ndoe, inner_node)
            no_cross = CreateRoadFrom(
                circular_1,
                1,
                road_1,
                self.block_network,
                self._global_network,
                center_line_type=LineType.NONE,
                side_lane_line_type=side_line_type,
                inner_lane_line_type=LineType.NONE
            ) and no_cross
            road_2 = Road(inner_node, self.road_node(0, 0))
            no_cross = CreateRoadFrom(
                circular_2,
                1,
                road_2,
                self.block_network,
                self._global_network,
                center_line_type=LineType.NONE,
                side_lane_line_type=side_line_type,
                inner_lane_line_type=LineType.NONE
            ) and no_cross

            # adverse part
            lane = negative_sockect_road.get_lanes(self.block_network)[-1]
            circular_2, circular_1, _ = create_wave_lanes(
                lane, lateral_dist, self.BOTTLENECK_LEN, 5, self.lane_width, False
            )
            road_2 = -road_2
            no_cross = CreateRoadFrom(
                circular_2,
                1,
                road_2,
                self.block_network,
                self._global_network,
                center_line_type=LineType.NONE,
                side_lane_line_type=side_line_type,
                inner_lane_line_type=LineType.NONE
            ) and no_cross

            road_1 = -road_1
            no_cross = CreateRoadFrom(
                circular_1,
                1,
                road_1,
                self.block_network,
                self._global_network,
                center_line_type=LineType.NONE,
                side_lane_line_type=side_line_type,
                inner_lane_line_type=LineType.NONE
            ) and no_cross

        return no_cross


class Split(Bottleneck):
    """
                        /-----
                       /
    -------------------
    -------------------
                       \
                        \-----
    OutBottlecneck
    """
    ID = "Y"

    def _try_plug_into_previous_block(self) -> bool:
        no_cross = True
        parameters = self.get_config()
        self.BOTTLENECK_LEN = parameters["bottle_len"]
        lane_num_changed = parameters[Parameter.lane_num]

        start_ndoe = self.pre_block_socket.positive_road.end_node
        straight_lane_num = self.positive_lane_num
        circular_lane_num = lane_num_changed
        total_num = straight_lane_num + circular_lane_num

        # part 1, straight road 0
        basic_lane = self.positive_lanes[straight_lane_num - 1]
        ref_lane = ExtendStraightLane(basic_lane, self.BOTTLENECK_LEN, [LineType.NONE, LineType.NONE])
        straight_road = Road(start_ndoe, self.road_node(0, 0))
        no_cross = CreateRoadFrom(
            ref_lane,
            straight_lane_num,
            straight_road,
            self.block_network,
            self._global_network,
            center_line_type=LineType.CONTINUOUS,
            side_lane_line_type=LineType.NONE,
            inner_lane_line_type=LineType.NONE
        ) and no_cross
        no_cross = CreateAdverseRoad(
            straight_road,
            self.block_network,
            self._global_network,
            inner_lane_line_type=LineType.NONE,
            side_lane_line_type=LineType.NONE,
            center_line_type=LineType.CONTINUOUS
        ) and no_cross

        # part 2, circular part
        lane = self.positive_lanes[-1]
        socket_road_ref_lane = None
        for index in range(1, circular_lane_num + 1):
            lateral_dist = index * self.lane_width / 2
            inner_node = self.road_node(1, index)
            side_line_type = LineType.SIDE if index == circular_lane_num else LineType.NONE

            # positive part
            circular_1, circular_2, straight = create_wave_lanes(
                lane, lateral_dist, self.BOTTLENECK_LEN, parameters[Parameter.length], self.lane_width, False
            )
            if index == circular_lane_num:
                socket_road_ref_lane = straight
            road_1 = Road(start_ndoe, inner_node)
            no_cross = CreateRoadFrom(
                circular_1,
                1,
                road_1,
                self.block_network,
                self._global_network,
                center_line_type=LineType.NONE,
                side_lane_line_type=side_line_type,
                inner_lane_line_type=LineType.NONE
            ) and no_cross
            road_2 = Road(inner_node, self.road_node(0, 0))
            no_cross = CreateRoadFrom(
                circular_2,
                1,
                road_2,
                self.block_network,
                self._global_network,
                center_line_type=LineType.NONE,
                side_lane_line_type=side_line_type,
                inner_lane_line_type=LineType.NONE
            ) and no_cross

        # extend for socket ,part 1 road 1
        socket_road = Road(self.road_node(0, 0), self.road_node(0, 1))
        no_cross = CreateRoadFrom(
            socket_road_ref_lane,
            total_num,
            socket_road,
            self.block_network,
            self._global_network,
            center_line_type=LineType.CONTINUOUS,
            side_lane_line_type=LineType.SIDE,
            inner_lane_line_type=LineType.BROKEN
        ) and no_cross
        no_cross = CreateAdverseRoad(
            socket_road,
            self.block_network,
            self._global_network,
            inner_lane_line_type=LineType.BROKEN,
            side_lane_line_type=LineType.SIDE,
            center_line_type=LineType.CONTINUOUS
        ) and no_cross

        negative_sockect_road = -socket_road
        self.add_sockets(PGBlockSocket(socket_road, negative_sockect_road))

        # part 2, circular part
        lanes = negative_sockect_road.get_lanes(self.block_network)
        for index, lane in enumerate(lanes[self.positive_lane_num:], 1):
            lateral_dist = index * self.lane_width / 2
            inner_node = self.road_node(1, index)
            side_line_type = LineType.SIDE if index == circular_lane_num else LineType.NONE

            # positive part
            circular_1, circular_2, _ = create_wave_lanes(lane, lateral_dist, self.BOTTLENECK_LEN, 5, self.lane_width)
            road_1 = -Road(inner_node, self.road_node(0, 0))

            no_cross = CreateRoadFrom(
                circular_1,
                1,
                road_1,
                self.block_network,
                self._global_network,
                center_line_type=LineType.NONE,
                side_lane_line_type=side_line_type,
                inner_lane_line_type=LineType.NONE
            ) and no_cross
            road_2 = -Road(start_ndoe, inner_node)
            no_cross = CreateRoadFrom(
                circular_2,
                1,
                road_2,
                self.block_network,
                self._global_network,
                center_line_type=LineType.NONE,
                side_lane_line_type=side_line_type,
                inner_lane_line_type=LineType.NONE
            ) and no_cross
        return no_cross
