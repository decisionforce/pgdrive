from typing import List

from pgdrive.pg_config import PgConfig
from pgdrive.pg_config.pg_blocks import PgBlock
from pgdrive.scene_creator.blocks.first_block import FirstBlock
from pgdrive.scene_creator.map import MapGenerateMethod, Map
from pgdrive.world.pg_world import PgWorld


class DynamicMap(Map):
    def __init__(self, pg_world: PgWorld, big_config: dict = None):
        """
        Map can be stored and recover to save time when we access the map encountered before
        """
        super(DynamicMap, self).__init__(pg_world, big_config)
        self.config = self.default_config()
        if big_config:
            self.config.update(big_config)
        self.lane_width = self.config[self.LANE_WIDTH]
        self.lane_num = self.config[self.LANE_NUM]
        self.random_seed = self.config[self.SEED]
        self.pg_world = pg_world

    def _generate_map(self, pg_world):
        """The old one,"""
        pass

    @staticmethod
    def default_config():
        return PgConfig(
            {
                Map.GENERATE_METHOD: MapGenerateMethod.BIG_BLOCK_NUM,
                Map.GENERATE_PARA: None,  # it can be a file path / block num / block ID sequence
                Map.LANE_WIDTH: 3.5,
                Map.LANE_NUM: 3,
                Map.SEED: 10
            }
        )

    def add_block(
        self,
        blocks_config: List,
    ):
        assert len(self.road_network.graph) == 0, "These Map is not empty, please create a new map to read config"
        last_block = FirstBlock(
            self.road_network, self.lane_width, self.lane_num, self.pg_world.worldNP, self.pg_world.physics_world, 1
        )
        self.blocks.append(last_block)
        for block_index, b in enumerate(blocks_config[1:], 1):
            block_type = PgBlock.get_block(b.pop(self.BLOCK_ID))
            pre_block_socket_index = b.pop(self.PRE_BLOCK_SOCKET_INDEX)
            last_block = block_type(
                block_index, last_block.get_socket(pre_block_socket_index), self.road_network, self.random_seed
            )
            last_block.construct_from_config(b, self.pg_world.worldNP, self.pg_world.physics_world)
            self.blocks.append(last_block)

    def refresh(self):
        self.road_network.update_indices()
        self.road_network.build_helper()
        self._load_to_highway_render(self.pg_world)
