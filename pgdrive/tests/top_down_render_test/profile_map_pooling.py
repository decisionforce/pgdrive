from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import setup_logger
import matplotlib.pyplot as plt

setup_logger(debug=True)


class TestEnv(PGDriveEnv):
    def __init__(self):
        super(TestEnv, self).__init__(
            {
                # "map_config": {
                #     Map.GENERATE_METHOD: MapGenerateMethod.BIG_BLOCK_SEQUENCE,
                #     Map.GENERATE_PARA: "OCrRCTXRCCCCrOr",
                #     Map.LANE_WIDTH: 3.5,
                #     Map.LANE_NUM: 3,
                # }
            }
        )


if __name__ == "__main__":
    env = TestEnv()
    env.reset()
    # it will return a numpy array
    plt.imshow(env.current_map.get_map_image_array(resolution=(64, 64)))
    plt.show()
