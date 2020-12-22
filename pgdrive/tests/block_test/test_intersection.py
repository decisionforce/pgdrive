from pgdrive.scene_creator.blocks.curve import Curve
from pgdrive.scene_creator.blocks.first_block import FirstBlock
from pgdrive.scene_creator.blocks.intersection import InterSection
from pgdrive.scene_creator.road.road_network import RoadNetwork
from pgdrive.tests.block_test.test_block_base import TestBlock

if __name__ == "__main__":
    test = TestBlock()
    from pgdrive.utils.asset_loader import AssetLoader

    AssetLoader.init_loader(test, test.asset_path)

    global_network = RoadNetwork()
    first = FirstBlock(global_network, 3.0, 2, test.render, test.world, 1)

    intersection = InterSection(3, first.get_socket(0), global_network, 1)
    print(intersection.construct_block(test.render, test.world))

    id = 4
    for socket_idx in range(intersection.SOCKET_NUM):
        block = Curve(id, intersection.get_socket(socket_idx), global_network, id)
        block.construct_block(test.render, test.world)
        id += 1
    test.run()
