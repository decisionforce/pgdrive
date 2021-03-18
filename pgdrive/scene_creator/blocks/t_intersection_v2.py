from pgdrive.scene_creator.blocks.t_intersection import TInterSection

from pgdrive.constants import Goal
from pgdrive.pg_config.parameter_space import Parameter, BlockParameterSpace
from pgdrive.pg_config.pg_space import PGSpace
from pgdrive.scene_creator.blocks.intersection import InterSection
from pgdrive.scene_creator.lane.abs_lane import LineType, LineColor
from pgdrive.scene_creator.road.road import Road
from pgdrive.constants import Decoration
from pgdrive.scene_creator.blocks.block import BlockSocket


class TInterSectionV2(TInterSection):

    ID = "TV2"
    SOCKET_NUM = 2
    PARAMETER_SPACE = PGSpace(BlockParameterSpace.T_INTERSECTION_V2)

    pass