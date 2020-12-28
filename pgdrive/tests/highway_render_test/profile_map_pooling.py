from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import setup_logger

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
    env.current_map.draw_map_image_on_surface(dest_resolution=(64, 64), pooling=0.5)
