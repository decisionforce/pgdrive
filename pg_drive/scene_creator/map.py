import json
import logging
import os
from typing import List

from panda3d.bullet import BulletWorld
from panda3d.core import NodePath

from pg_drive.pg_config.pg_blocks import PgBlock
from pg_drive.pg_config.pg_config import PgConfig
from pg_drive.scene_creator.algorithm.BIG import BIG, BigGenerateMethod
from pg_drive.scene_creator.blocks.block import Block
from pg_drive.scene_creator.road.road_network import RoadNetwork
import copy


class MapGenerateMethod:
    BIG_BLOCK_NUM = BigGenerateMethod.BLOCK_NUM
    BIG_BLOCK_SEQUENCE = BigGenerateMethod.BLOCK_SEQUENCE
    PG_MAP_FILE = "pg_map_file"


class Map:
    # only used to save and read maps
    FILE_SUFFIX = ".json"

    # define string in json and config
    SEED = "seed"
    LANE_WIDTH = "lane_width"
    LANE_NUM = "lane_num"
    BLOCK_ID = "id"
    BLOCK_SEQUENCE = "block_sequence"
    PRE_BLOCK_SOCKET_INDEX = "pre_block_socket_index"

    # generate_method
    GENERATE_PARA = "config"
    GENERATE_METHOD = "type"

    def __init__(self, parent_node_path: NodePath, pg_physics_world: BulletWorld, big_config: dict = None):
        """
        Map can be stored and recover to save time when we access the map encountered before
        """
        self.config = self.default_config()
        if big_config:
            self.config.update(big_config)
        self.lane_width = self.config[self.LANE_WIDTH]
        self.lane_num = self.config[self.LANE_NUM]
        self.random_seed = self.config[self.SEED]
        self.road_network = RoadNetwork()
        self.blocks = []
        generate_type = self.config[self.GENERATE_METHOD]
        if generate_type == BigGenerateMethod.BLOCK_NUM or generate_type == BigGenerateMethod.BLOCK_SEQUENCE:
            self._big_generate(parent_node_path, pg_physics_world)

        elif generate_type == MapGenerateMethod.PG_MAP_FILE:
            # other config such as lane width, num and seed will be valid, since they will be read from file
            blocks_config = self.read_map(self.config[self.GENERATE_PARA])
            self._config_generate(blocks_config, parent_node_path, pg_physics_world)
            # print("Loading map from json file!")

        else:
            raise ValueError("Map can not be created by {}".format(generate_type))

        #  a trick to optimize performance
        self.road_network.update_indices()
        self.road_network.build_helper()

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

    def _big_generate(self, parent_node_path: NodePath, pg_physics_world: BulletWorld):
        big_map = BIG(
            self.lane_num, self.lane_width, self.road_network, parent_node_path, pg_physics_world, self.random_seed
        )
        big_map.generate(self.config[self.GENERATE_METHOD], self.config[self.GENERATE_PARA])
        self.blocks = big_map.blocks

    def _config_generate(self, blocks_config: List, parent_node_path: NodePath, pg_physics_world: BulletWorld):
        assert len(self.road_network.graph) == 0, "These Map is not empty, please create a new map to read config"
        from pg_drive.scene_creator.blocks.first_block import FirstBlock
        last_block = FirstBlock(
            self.road_network, self.lane_width, self.lane_num, parent_node_path, pg_physics_world, 1
        )
        self.blocks.append(last_block)
        for block_index, b in enumerate(blocks_config[1:], 1):
            block_type = PgBlock.get_block(b.pop(self.BLOCK_ID))
            pre_block_socket_inex = b.pop(self.PRE_BLOCK_SOCKET_INDEX)
            last_block = block_type(
                block_index, last_block.get_socket(pre_block_socket_inex), self.road_network, self.random_seed
            )
            last_block.construct_from_config(b, parent_node_path, pg_physics_world)
            self.blocks.append(last_block)

    def re_generate(self, parent_node_path: NodePath, pg_physics_world: BulletWorld):
        """
        For convenience
        """
        self.add_to_physics_world(pg_physics_world)
        from pg_drive.utils.visualization_loader import VisLoader
        if VisLoader.loader is not None:
            self.add_to_render_module(parent_node_path)

    def add_to_render_module(self, parent_node_path: NodePath):
        """
        If original node path is removed, this can re attach blocks to render module
        """
        for block in self.blocks:
            block.add_to_render_module(parent_node_path)

    def add_to_physics_world(self, pg_physics_world: BulletWorld):
        """
        If the original bullet physics world is deleted, call this to re-add road network
        """
        for block in self.blocks:
            block.add_to_physics_world(pg_physics_world)

    def remove_from_physics_world(self, pg_physics_world: BulletWorld):
        for block in self.blocks:
            block.remove_from_physics_world(pg_physics_world)
        # print("Current map has blocks: ", len(self.blocks))

    def remove_from_render_module(self):
        for block in self.blocks:
            block.remove_from_render_module()

    def destroy_map(self, pg_physics_world: BulletWorld):
        for block in self.blocks:
            block.destroy(pg_physics_world)

    def save_map(self):
        """
        This func will generate a json file named 'map_name.json', in 'save_dir'
        """
        assert self.blocks is not None and len(self.blocks) > 0, "Please generate Map before saving it"
        import numpy as np
        map_config = []
        for b in self.blocks:
            assert isinstance(b, Block), "None Block type can not be saved to json file"
            b_config = b.get_config()
            json_config = {}
            for k, v in b_config._config.items():
                json_config[k] = v.tolist()[0] if isinstance(v, np.ndarray) else v
            json_config[self.BLOCK_ID] = b.ID
            json_config[self.PRE_BLOCK_SOCKET_INDEX] = b.pre_block_socket_index
            map_config.append(json_config)

        saved_data = copy.deepcopy(
            {
                self.SEED: self.random_seed,
                self.LANE_NUM: self.lane_num,
                self.LANE_WIDTH: self.lane_width,
                self.BLOCK_SEQUENCE: map_config
            }
        )
        return saved_data

    def save_map_to_json(self, map_name: str, save_dir: str = os.path.dirname(__file__)):
        data = self.save_map()
        with open(os.path.join(save_dir, map_name + self.FILE_SUFFIX), 'w') as outfile:
            json.dump(data, outfile)

    def read_map(self, map_config: dict):
        """
        Load the map from a dict
        """
        self.config[self.LANE_NUM] = map_config[self.LANE_NUM]
        self.config[self.LANE_WIDTH] = map_config[self.LANE_WIDTH]
        self.config[self.SEED] = map_config[self.SEED]
        blocks_config = map_config[self.BLOCK_SEQUENCE]

        # update the property
        self.lane_width = self.config[self.LANE_WIDTH]
        self.lane_num = self.config[self.LANE_NUM]
        self.random_seed = self.config[self.SEED]
        return blocks_config

    def read_map_from_json(self, map_file_path: str):
        """
        Create map from a .json file, read it to map config and update default properties
        """
        with open(map_file_path, "r") as map_file:
            map_config = json.load(map_file)
            ret = self.read_map(map_config)
        return ret

    def __del__(self):
        describe = self.random_seed if self.random_seed is not None else "custom"
        logging.debug("Scene {} is destroyed".format(describe))
