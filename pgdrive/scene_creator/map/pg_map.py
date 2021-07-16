from typing import List

from panda3d.core import NodePath

from pgdrive.engine.core.pg_physics_world import PGPhysicsWorld
from pgdrive.scene_creator.algorithm.BIG import BigGenerateMethod, BIG
from pgdrive.scene_creator.algorithm.blocks_prob_dist import PGBlock
from pgdrive.scene_creator.blocks import FirstBlock
from pgdrive.scene_creator.map.map import Map, MapGenerateMethod


class PGMap(Map):
    def _generate(self):
        """
        We can override this function to introduce other methods!
        """
        parent_node_path, pg_physics_world = self.pgdrive_engine.worldNP, self.pgdrive_engine.physics_world
        generate_type = self.config[self.GENERATE_TYPE]
        if generate_type == BigGenerateMethod.BLOCK_NUM or generate_type == BigGenerateMethod.BLOCK_SEQUENCE:
            self._big_generate(parent_node_path, pg_physics_world)

        elif generate_type == MapGenerateMethod.PG_MAP_FILE:
            # other config such as lane width, num and seed will be valid, since they will be read from file
            blocks_config = self.read_map(self.config[self.GENERATE_CONFIG])
            self._config_generate(blocks_config, parent_node_path, pg_physics_world)
        else:
            raise ValueError("Map can not be created by {}".format(generate_type))

    def _big_generate(self, parent_node_path: NodePath, pg_physics_world: PGPhysicsWorld):
        big_map = BIG(
            self.config[self.LANE_NUM],
            self.config[self.LANE_WIDTH],
            self.road_network,
            parent_node_path,
            pg_physics_world,
            self.config["block_type_version"],
            exit_length=self.config["exit_length"]
        )
        big_map.generate(self.config[self.GENERATE_TYPE], self.config[self.GENERATE_CONFIG])
        self.blocks = big_map.blocks

    def _config_generate(self, blocks_config: List, parent_node_path: NodePath, pg_physics_world: PGPhysicsWorld):
        assert len(self.road_network.graph) == 0, "These Map is not empty, please create a new map to read config"
        last_block = FirstBlock(
            self.road_network, self.config[self.LANE_WIDTH], self.config[self.LANE_NUM], parent_node_path,
            pg_physics_world, 1
        )
        self.blocks.append(last_block)
        for block_index, b in enumerate(blocks_config[1:], 1):
            block_type = PGBlock.get_block(b.pop(self.BLOCK_ID), self.config["block_type_version"])
            pre_block_socket_index = b.pop(self.PRE_BLOCK_SOCKET_INDEX)
            last_block = block_type(
                block_index, last_block.get_socket(pre_block_socket_index), self.road_network)
            last_block.construct_from_config(b, parent_node_path, pg_physics_world)
            self.blocks.append(last_block)