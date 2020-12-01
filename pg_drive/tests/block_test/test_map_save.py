from pg_drive.scene_creator.algorithm.BIG import BigGenerateMethod
from pg_drive.scene_creator.road.road_network import RoadNetwork
from pg_drive.tests.block_test.test_block_base import TestBlock
from pg_drive.utils.visualization_loader import VisLoader
from pg_drive.scene_creator.map import Map
import os

if __name__ == "__main__":
    test = TestBlock()
    VisLoader.init_loader(test.loader, test.asset_path)
    map = Map(big_config=dict(type=BigGenerateMethod.BLOCK_NUM, config=12))
    map._big_generate(3.5, 2, 0, parent_node_path=test.render, physics_world=test.physics_world)
    map.save_map("test_save", os.path.dirname(__file__))