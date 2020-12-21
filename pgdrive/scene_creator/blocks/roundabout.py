import numpy as np

from pgdrive.pg_config.parameter_space import Parameter, BlockParameterSpace
from pgdrive.pg_config.pg_space import PgSpace
from pgdrive.scene_creator.basic_utils import CreateAdverseRoad, CreateRoadFrom, sharpbend
from pgdrive.scene_creator.blocks.block import Block, BlockSocket
from pgdrive.scene_creator.lanes.lane import LineType
from pgdrive.scene_creator.lanes.straight_lane import StraightLane
from pgdrive.scene_creator.road.road import Road


class Roundabout(Block):
    """
    roundabout class, the example is the same as Intersection
    """
    ID = "O"
    PARAMETER_SPACE = PgSpace(BlockParameterSpace.ROUNDABOUT)
    SOCKET_NUM = 3
    RADIUS_IN = 20
    ANGLE = 60
    EXIT_PART_LENGTH = 30

    def _try_plug_into_previous_block(self) -> bool:
        para = self.get_config()
        no_cross = True
        attach_road = self._pre_block_socket.positive_road
        for i in range(4):
            exit_road, success = self._create_circular_part(
                attach_road, i, para[Parameter.radius_exit], para[Parameter.radius_inner], para[Parameter.angle]
            )
            no_cross = no_cross and success
            if i < 3:
                no_cross = CreateAdverseRoad(exit_road, self.block_network, self._global_network) and no_cross
                attach_road = -exit_road
        self.add_reborn_roads([socket.negative_road for socket in self._sockets])
        return no_cross

    def _create_circular_part(self, road: Road, part_idx: int, radius_exit: float, radius_inner: float,
                              angle: float) -> (str, str, StraightLane, bool):
        """
        Create a part of roundabout according to a straight road
        """
        none_cross = True
        self.set_part_idx(part_idx)
        radius_big = (self.positive_lane_num * 2 - 1) * self.lane_width + radius_inner

        # circular part 0
        segment_start_node = road.end_node
        segment_end_node = self.add_road_node()
        segment_road = Road(segment_start_node, segment_end_node)
        lanes = road.get_lanes(self._global_network) if part_idx == 0 else road.get_lanes(self.block_network)
        right_lane = lanes[-1]
        bend, straight = sharpbend(
            right_lane, 10, radius_exit, np.deg2rad(angle), True, self.lane_width, (LineType.STRIPED, LineType.SIDE)
        )
        ignore_last_2_part_start = self.road_node((part_idx + 3) % 4, 0)
        ignore_last_2_part_end = self.road_node((part_idx + 3) % 4, 0)
        none_cross = CreateRoadFrom(
            bend,
            self.positive_lane_num,
            segment_road,
            self.block_network,
            self._global_network,
            ignore_start=ignore_last_2_part_start,
            ignore_end=ignore_last_2_part_end
        ) and none_cross

        # set circular part 0 visualization
        for k, lane in enumerate(segment_road.get_lanes(self.block_network)):
            if k == self.positive_lane_num - 1:
                lane.line_types = [LineType.NONE, LineType.SIDE]
            else:
                lane.line_types = [LineType.NONE, LineType.NONE]

        # circular part 1
        tool_lane_start = straight.position(-5, 0)
        tool_lane_end = straight.position(0, 0)
        tool_lane = StraightLane(tool_lane_start, tool_lane_end)

        bend, straight_to_next_iter_part = sharpbend(
            tool_lane, 10, radius_big, np.deg2rad(2 * angle - 90), False, self.lane_width,
            (LineType.STRIPED, LineType.SIDE)
        )

        segment_start_node = segment_end_node
        segment_end_node = self.add_road_node()
        segment_road = Road(segment_start_node, segment_end_node)

        none_cross = CreateRoadFrom(
            bend, self.positive_lane_num, segment_road, self.block_network, self._global_network
        ) and none_cross

        # circular part 2 and exit straight part
        length = self.EXIT_PART_LENGTH
        tool_lane_start = straight_to_next_iter_part.position(-5, 0)
        tool_lane_end = straight_to_next_iter_part.position(0, 0)
        tool_lane = StraightLane(tool_lane_start, tool_lane_end)

        bend, straight = sharpbend(
            tool_lane, length, radius_exit, np.deg2rad(angle), True, self.lane_width, (LineType.STRIPED, LineType.SIDE)
        )

        segment_start_node = segment_end_node
        segment_end_node = self.add_road_node()
        segment_road = Road(segment_start_node, segment_end_node)

        none_cross = CreateRoadFrom(
            bend, self.positive_lane_num, segment_road, self.block_network, self._global_network
        ) and none_cross

        # set circular part 2 (curve) visualization
        for k, lane in enumerate(segment_road.get_lanes(self.block_network)):
            if k == self.positive_lane_num - 1:
                lane.line_types = [LineType.NONE, LineType.SIDE]
            else:
                lane.line_types = [LineType.NONE, LineType.NONE]

        exit_start = segment_end_node
        exit_end = self.add_road_node()
        segment_road = Road(exit_start, exit_end)
        if part_idx < 3:
            none_cross = CreateRoadFrom(
                straight, self.positive_lane_num, segment_road, self.block_network, self._global_network
            ) and none_cross
            self.add_sockets(self.create_socket_from_positive_road(segment_road))

        #  add circular part 3 at last
        segment_start = self.road_node(part_idx, 1)
        segment_end = self.road_node((part_idx + 1) % 4, 0)
        segment_road = Road(segment_start, segment_end)
        tool_lane_start = straight_to_next_iter_part.position(-6, 0)
        tool_lane_end = straight_to_next_iter_part.position(0, 0)
        tool_lane = StraightLane(tool_lane_start, tool_lane_end)

        beneath = (self.positive_lane_num * 2 - 1) * self.lane_width / 2 + radius_exit
        cos = np.cos(np.deg2rad(angle))
        radius_this_seg = beneath / cos - radius_exit

        bend, _ = sharpbend(
            tool_lane, 5, radius_this_seg, np.deg2rad(180 - 2 * angle), False, self.lane_width,
            (LineType.STRIPED, LineType.SIDE)
        )
        CreateRoadFrom(bend, self.positive_lane_num, segment_road, self.block_network, self._global_network)

        # set circular part 2 visualization
        for k, lane in enumerate(segment_road.get_lanes(self.block_network)):
            if k == 0:
                if self.positive_lane_num > 1:
                    lane.line_types = [LineType.CONTINUOUS, LineType.STRIPED]
                else:
                    lane.line_types = [LineType.CONTINUOUS, LineType.NONE]
            else:
                lane.line_types = [LineType.STRIPED, LineType.STRIPED]

        return Road(exit_start, exit_end), none_cross

    def get_socket(self, index: int) -> BlockSocket:
        socket = super(Roundabout, self).get_socket(index)
        self._reborn_roads.remove(socket.negative_road)
        return socket
