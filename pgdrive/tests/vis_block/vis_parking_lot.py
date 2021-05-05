from pgdrive.scene_creator.blocks.first_block import FirstBlock
from pgdrive.scene_creator.blocks.parking_lot import ParkingLot
from pgdrive.scene_creator.road.road_network import RoadNetwork
from pgdrive.tests.vis_block.vis_block_base import TestBlock

if __name__ == "__main__":
    test = TestBlock()
    from pgdrive.utils.asset_loader import initialize_asset_loader

    initialize_asset_loader(test)

    global_network = RoadNetwork()
    last = FirstBlock(global_network, 3, 1, test.render, test.world, 1)

    for i in range(1, 3):
        last = ParkingLot(i, last.get_socket(0), global_network, 1)
        print(last.construct_block(test.render, test.world))

    # test.show_bounding_box(global_network)
    test.run()
