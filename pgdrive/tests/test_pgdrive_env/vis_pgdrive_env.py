from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.scene_creator.map import Map, MapGenerateMethod
from pgdrive.utils import setup_logger

setup_logger(False)


class TestEnv(PGDriveEnv):
    def __init__(self):
        super(TestEnv, self).__init__(
            {"map": 20,
             "environment_num": 1,
             "pg_world_config": {"pstats": True}
             }
        )


if __name__ == "__main__":
    env = TestEnv()

    o = env.reset()
    for i in range(1, 1000000):
        o, r, d, info = env.step([0, 0])
        # env.render()
        # if d:
        #     print("Reset")
        #     env.reset()
    env.close()
