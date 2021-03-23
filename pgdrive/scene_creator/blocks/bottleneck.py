import numpy as np

from pgdrive.pg_config.parameter_space import BlockParameterSpace, Parameter
from pgdrive.pg_config.pg_space import PGSpace
from pgdrive.scene_creator.blocks.block import Block
from pgdrive.scene_creator.blocks.create_block_utils import CreateAdverseRoad, CreateRoadFrom, sharpbend, ExtendStraightLane
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
    AGNLE = 45  # degree


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
        parameters = self.get_config()
        lane_num_changed = parameters[Parameter.lane_num]

        start_ndoe = self.pre_block_socket.positive_road.end_node
        straight_lane_num = self.positive_lane_num - lane_num_changed
        straight_lane_num = max(1, straight_lane_num)

        circular_lane_num = self.positive_lane_num - straight_lane_num

        # part 1, straight part
        assert 0 < self.AGNLE < 90
        straight_len = np.tan(np.deg2rad(self.AGNLE)) * self.lane_width
        basic_lane = self.positive_lanes[:straight_lane_num-1]
        ref_lane = ExtendStraightLane(basic_lane, straight_len, [LineType.NONE, LineType.NONE])




class InBottleneck(Bottleneck):
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
