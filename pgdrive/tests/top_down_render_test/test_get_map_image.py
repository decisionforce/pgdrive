from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import setup_logger


def test_save_map_image():
    setup_logger(debug=True)
    env = PGDriveEnv()
    env.reset()
    env.current_map.save_map_image(resolution=(64, 64))
    env.close()


if __name__ == "__main__":
    test_save_map_image()
