import json
import logging
import os

from panda3d.bullet import BulletWorld
from panda3d.core import NodePath
from pg_drive.pg_config.pg_blocks import PgBlock
from pg_drive.pg_config.pg_config import PgConfig
from pg_drive.scene_creator.algorithm.BIG import BIG, BigGenerateMethod
from pg_drive.scene_creator.blocks.block import Block
from pg_drive.scene_creator.road.road_network import RoadNetwork


class Map:
    # only used to save and read maps
    FILE_SUFFIX = ".pgm"
    MAP_SEED = "seed"
    MAP_LANE_WIDTH = "lane_width"
    MAP_LANE_NUM = "lane_num"
    BLOCK_ID = "id"
    BLOCK_SEQUENCE = "block_sequence"
    PRE_BLOCK_SOCKET_INDEX = "pre_block_socket_index"

    def __init__(self, big_config: dict = None):
        """
        Scene can be stored and recover to save time when we access scenes encountered before
        Scene should contain road_network, blocks and vehicles
        """
        self.road_network = RoadNetwork()
        self.blocks = []
        self.lane_width = None
        self.lane_num = None
        self.random_seed = None
        self.config = self.default_config()
        if big_config:
            self.config.update(big_config)

    @staticmethod
    def default_config():
        return PgConfig({"type": BigGenerateMethod.BLOCK_NUM, "config": None})

    def big_generate(
            self, lane_width: float, lane_num: int, seed: int, parent_node_path: NodePath, physics_world: BulletWorld
    ):
        map = BIG(lane_num, lane_width, self.road_network, parent_node_path, physics_world, seed)
        map.generate(self.config["type"], self.config["config"])
        self.random_seed = seed
        self.blocks = map.blocks
        self.lane_num = lane_num
        self.lane_width = lane_width
        # TODO wrap this to support more generating methods
        self.road_network.update_indices()
        self.road_network.build_helper()

    def config_generate(self, map_file_path: str, parent_node_path: NodePath, physics_world: BulletWorld):
        assert len(self.road_network.graph) == 0, "These Map is not empty, please create a new map to read config"
        blocks_config = self.read_map(map_file_path)
        from pg_drive.scene_creator.blocks.first_block import FirstBlock
        last_block = FirstBlock(self.road_network, self.lane_width, self.lane_num, parent_node_path, physics_world, 1)
        for block_index, b in enumerate(blocks_config[1:], 1):
            block_type = PgBlock.get_block(b.pop(self.BLOCK_ID))
            pre_block_socket_inex = b.pop(self.PRE_BLOCK_SOCKET_INDEX)
            last_block = block_type(block_index, last_block.get_socket(pre_block_socket_inex), self.road_network,
                                    self.random_seed)
            last_block.construct_from_config(b, parent_node_path, physics_world)

    def re_generate(self, parent_node_path: NodePath, bt_physics_world: BulletWorld):
        """
        For convenience
        """
        self.add_to_bullet_physics_world(bt_physics_world)
        from pg_drive.utils.visualization_loader import VisLoader
        if VisLoader.loader is not None:
            self.add_to_render_module(parent_node_path)

    def add_to_render_module(self, parent_node_path: NodePath):
        """
        If original node path is removed, this can re attach blocks to render module
        """
        for block in self.blocks:
            block.add_to_render_module(parent_node_path)

    def add_to_bullet_physics_world(self, bt_physics_world: BulletWorld):
        """
        If the original bullet physics world is deleted, call this to re-add road network
        """
        for block in self.blocks:
            block.add_to_physics_world(bt_physics_world)

    def remove_from_physics_world(self, bt_physics_world: BulletWorld):
        for block in self.blocks:
            block.remove_from_physics_world(bt_physics_world)

    def remove_from_render_module(self):
        for block in self.blocks:
            block.remove_from_render_module()

    def destroy_map(self, bt_physics_world: BulletWorld):
        for block in self.blocks:
            block.destroy(bt_physics_world)

    def save_map(self, map_name: str, save_dir: str = os.path.dirname(__file__)):
        """
        This func will generate a json file named 'map_name.pgm', in 'save_dir'
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
        with open(os.path.join(save_dir, map_name + self.FILE_SUFFIX), 'w') as outfile:
            json.dump({self.MAP_SEED: self.random_seed,
                       self.MAP_LANE_NUM: self.lane_num,
                       self.MAP_LANE_WIDTH: self.lane_width,
                       self.BLOCK_SEQUENCE: map_config}, outfile)

    def read_map(self, map_file_path: str):
        """
        Create map from a .pgm file
        """
        with open(map_file_path, "r") as map_file:
            map_config = json.load(map_file)
            self.lane_num = map_config[self.MAP_LANE_NUM]
            self.lane_width = map_config[self.MAP_LANE_WIDTH]
            self.random_seed = map_config[self.MAP_SEED]
            blocks_config = map_config[self.BLOCK_SEQUENCE]
        return blocks_config

    def __del__(self):
        describe = self.random_seed if self.random_seed is not None else "custom"
        logging.debug("Scene {} is destroyed".format(describe))
