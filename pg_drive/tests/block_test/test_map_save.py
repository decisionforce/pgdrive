from pg_drive.tests.block_test.test_block_base import TestBlock
from pg_drive.scene_creator.map import MapGenerateMethod, Map

from pg_drive.utils.visualization_loader import VisLoader
from pg_drive.scene_creator.map import Map
import os

if __name__ == "__main__":
    test = TestBlock()
    VisLoader.init_loader(test.loader, test.asset_path)
    map = Map(test.render, test.physics_world,
              big_config={Map.GENERATE_METHOD: MapGenerateMethod.BIG_BLOCK_NUM, Map.GENERATE_PARA: 12})
    map.save_map("test_save", os.path.dirname(__file__))