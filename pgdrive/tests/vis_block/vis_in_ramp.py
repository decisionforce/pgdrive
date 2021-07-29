from pgdrive.component.blocks.first_block import FirstPGBlock
from pgdrive.component.blocks.ramp import InRampOnStraight
from pgdrive.component.blocks.straight import Straight
from pgdrive.component.road.road_network import RoadNetwork
from pgdrive.tests.vis_block.vis_block_base import TestBlock
from pgdrive.engine.asset_loader import initialize_asset_loader

if __name__ == "__main__":

    test = TestBlock()

    initialize_asset_loader(test)

    global_network = RoadNetwork()
    straight = FirstPGBlock(global_network, 3.0, 1, test.render, test.world, 1)
    straight = Straight(4, straight.get_socket(0), global_network, 1)
    print(straight.construct_block(test.render, test.world))
    print(len(straight.dynamic_nodes))
    for i in range(1, 3):
        straight = InRampOnStraight(i, straight.get_socket(0), global_network, i)
        print(straight.construct_block(test.render, test.world))
        print(len(straight.dynamic_nodes))
    test.show_bounding_box(global_network)
    test.run()
