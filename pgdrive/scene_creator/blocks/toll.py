from pgdrive.scene_creator.blocks.block import Block, BlockSocket
from pgdrive.scene_creator.blocks.create_block_utils import CreateAdverseRoad, CreateRoadFrom, ExtendStraightLane, \
    create_wave_lanes
from pgdrive.scene_creator.lane.abs_lane import LineType
from pgdrive.scene_creator.road.road import Road
from pgdrive.utils.pg_space import PGSpace, Parameter, BlockParameterSpace
from pgdrive.scene_creator.blocks.bottleneck import Bottleneck

class Toll(Bottleneck):
    """
    -----\
          \
           -------------------
           -------------------
          /
    -----/
    Toll, like merge block
    """
    ID = "$"

    def _try_plug_into_previous_block(self) -> bool:
        pass

