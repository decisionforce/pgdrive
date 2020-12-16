from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import setup_logger

setup_logger(debug=True)


class TestEnv(PGDriveEnv):
    def __init__(self):
        super(TestEnv, self).__init__({
            "use_render": False,
            "use_image": False,
        })


if __name__ == "__main__":
    env = TestEnv()

    env.reset()
    for i in range(1, 100):
        o, r, d, info = env.step([0, 1])
    env.close()
    print("Physics world successfully run!")
