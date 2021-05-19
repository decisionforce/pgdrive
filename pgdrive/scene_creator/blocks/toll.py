from pgdrive.scene_creator.blocks.block import Block, BlockSocket
import numpy as np
from pgdrive.scene_creator.blocks.create_block_utils import CreateAdverseRoad, CreateRoadFrom, ExtendStraightLane, \
    create_wave_lanes
from pgdrive.scene_creator.lane.abs_lane import LineType, LineColor
from pgdrive.scene_creator.road.road import Road
from pgdrive.utils.pg_space import PGSpace, Parameter, BlockParameterSpace
from pgdrive.scene_creator.blocks.bottleneck import Block

from pgdrive.scene_creator.object.base_object import BaseObject

class TollBuilding(BaseObject):

class Toll(Block):
    """
    Toll, like Straight, but has speed limit
    """
    SOCKET_NUM = 1
    PARAMETER_SPACE = PGSpace(BlockParameterSpace.BOTTLENECK_PARAMETER)
    ID = "$"

    BUILDING_LENGTH = 5

    def _try_plug_into_previous_block(self) -> bool:
        self.set_part_idx(0)  # only one part in simple block like straight, and curve
        para = self.get_config()
        length = para[Parameter.length]
        basic_lane = self.positive_basic_lane
        new_lane = ExtendStraightLane(basic_lane, length, [LineType.CONTINUOUS, LineType.SIDE])
        start = self.pre_block_socket.positive_road.end_node
        end = self.add_road_node()
        socket = Road(start, end)
        _socket = -socket

        # create positive road
        no_cross = CreateRoadFrom(new_lane, self.positive_lane_num, socket, self.block_network, self._global_network,
                                  center_line_color=LineColor.GREY,
                                  center_line_type=LineType.CONTINUOUS,
                                  inner_lane_line_type=LineType.CONTINUOUS,
                                  side_lane_line_type=LineType.SIDE)

        # create negative road
        no_cross = CreateAdverseRoad(socket, self.block_network, self._global_network,
                                     center_line_color=LineColor.GREY,
                                     center_line_type=LineType.CONTINUOUS,
                                     inner_lane_line_type=LineType.CONTINUOUS,
                                     side_lane_line_type=LineType.SIDE
                                     ) and no_cross

        self.add_sockets(BlockSocket(socket, _socket))
        return no_cross

    def _add_building(self, road):
        # add house
        lanes = road.get_lanes(self.block_network)
        for idx, lane in enumerate(lanes):
            if idx % 2 == 1:
                # add toll
                position = lane.position(lane.length / 2, 0)
                node_path = self._add_invisible_static_wall(position, np.rad2deg(lane.heading_at(0)),
                                                            self.BUILDING_LENGTH,
                                                            self.lane_width, 3.5, name="Toll")

    def add_buildings(self):
        socket = self.get_socket(index=0)
        for road in [socket.positive_road, socket.negative_road]:
            node_path = self._add_building(road)
