import numpy as np
from pgdrive.scene_creator.lane.straight_lane import StraightLane
from pgdrive.utils.scene_utils import check_lane_on_road
from pgdrive.pg_config.parameter_space import BlockParameterSpace, Parameter
from pgdrive.pg_config.pg_space import PGSpace
from pgdrive.scene_creator.blocks.block import Block, BlockSocket
from pgdrive.scene_creator.blocks.create_block_utils import CreateAdverseRoad, CreateRoadFrom, sharpbend, \
    ExtendStraightLane
from pgdrive.scene_creator.lane.abs_lane import LineType
from pgdrive.scene_creator.road.road import Road


class Bottleneck(Block):
    """
    This block is used to change thr lane num
    """
    ID = None
    SOCKET_NUM = 1
    PARAMETER_SPACE = PGSpace(BlockParameterSpace.BOTTLENECK_PARAMETER)

    # property of bottleneck
    BOTTLENECK_LEN = 20  # Add to parameter sapce in the future

    @staticmethod
    def _wave_lanes(pre_lane, lateral_dist: float, length: float, lane_width, toward_left=True):
        """
        Prodeuce two lanes in adverse direction
        :param pre_lane: Previous abstract lane
        :param lateral_dist: the dist moved in previous lane's lateral direction
        :param length: the length in the previous lane's longitude direction
        :return: List[Circular lane]
        """
        angle = np.pi - 2 * np.arctan(length / (2 * lateral_dist))
        radius = length / (2 * np.sin(angle))
        circular_lane_1, pre_lane = sharpbend(pre_lane, 10, radius, angle, False if toward_left else True,
                                              lane_width,
                                              [LineType.NONE, LineType.NONE])
        pre_lane.reset_start_end(pre_lane.position(-10, 0), pre_lane.position(pre_lane.length - 10, 0))
        circular_lane_2, _ = sharpbend(pre_lane, 5, radius, angle, True if toward_left else True, lane_width,
                                       [LineType.NONE, LineType.NONE])
        return circular_lane_1, circular_lane_2


class InBottleneck(Bottleneck):
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
        lane_num_changed = 2

        start_ndoe = self.pre_block_socket.positive_road.end_node
        straight_lane_num = int(self.positive_lane_num - lane_num_changed)
        straight_lane_num = max(1, straight_lane_num)

        circular_lane_num = self.positive_lane_num - straight_lane_num

        # part 1, straight part
        basic_lane = self.positive_lanes[straight_lane_num - 1]
        ref_lane = ExtendStraightLane(basic_lane, self.BOTTLENECK_LEN, [LineType.NONE, LineType.NONE])
        straight_road = Road(start_ndoe, self.road_node(0, 0))
        no_cross = CreateRoadFrom(ref_lane, straight_lane_num, straight_road, self.block_network, self._global_network,
                                  center_line_type=LineType.CONTINUOUS, side_lane_line_type=LineType.NONE,
                                  inner_lane_line_type=LineType.NONE) and no_cross
        no_cross = CreateAdverseRoad(straight_road, self.block_network, self._global_network,
                                     inner_lane_line_type=LineType.NONE,
                                     side_lane_line_type=LineType.NONE,
                                     center_line_type=LineType.CONTINUOUS) and no_cross

        # part 2, circular part
        for index, lane in enumerate(self.positive_lanes[straight_lane_num:], 1):
            lateral_dist = index * self.lane_width / 2

            circular_1, circular_2 = self._wave_lanes(lane, lateral_dist, self.BOTTLENECK_LEN, self.lane_width)
            side_line_type = LineType.SIDE if index == self.positive_lane_num - straight_lane_num else LineType.NONE
            road = Road(start_ndoe, self.road_node(1, index))
            no_cross = CreateRoadFrom(circular_1, 1, road, self.block_network,
                                      self._global_network,
                                      center_line_type=LineType.NONE,
                                      side_lane_line_type=side_line_type,
                                      inner_lane_line_type=LineType.NONE) and no_cross
            road = Road(start_ndoe, self.road_node(2, index))
            no_cross = CreateRoadFrom(circular_2, 1, road, self.block_network,
                                      self._global_network,
                                      center_line_type=LineType.NONE,
                                      side_lane_line_type=side_line_type,
                                      inner_lane_line_type=LineType.NONE) and no_cross

        # FIXME
        self.add_sockets(BlockSocket(straight_road, -straight_road))
        return no_cross


class OutBottleneck(Bottleneck):
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
        parameters = self.get_config()
