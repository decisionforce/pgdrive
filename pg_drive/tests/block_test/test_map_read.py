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
    map.config_generate(os.path.join("./", "test_save.pgm"), test.render, test.physics_world)
    # test.run()
