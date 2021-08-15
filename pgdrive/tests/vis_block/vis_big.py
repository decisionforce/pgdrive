from pgdrive.component.algorithm.BIG import BIG
from pgdrive.component.road.road_network import RoadNetwork
from pgdrive.engine.asset_loader import initialize_asset_loader
from pgdrive.engine.engine_utils import set_global_random_seed
from pgdrive.tests.vis_block.vis_block_base import TestBlock


def vis_big(debug: bool = False):
    test = TestBlock(debug=debug)

    test.cam.setPos(300, 400, 2000)

    initialize_asset_loader(test)
    set_global_random_seed(4)
    global_network = RoadNetwork()

    big = BIG(2, 3.5, global_network, test.render, test.world)
    test.vis_big(big)
    test.big.block_num = 40
    # big.generate(BigGenerateMethod.BLOCK_NUM, 10)
    test.run()


if __name__ == "__main__":
    vis_big()
