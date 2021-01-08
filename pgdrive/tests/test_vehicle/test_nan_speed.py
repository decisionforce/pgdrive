import numpy as np
from pgdrive.envs.pgdrive_env import PGDriveEnv


def test_nan_speed():
    env = PGDriveEnv({"environment_num": 1, "traffic_density": 0.0})
    steering = [-np.nan, -1, 0, 1, np.nan]
    acc_brake = [-np.nan, -1, 0, 1, np.nan]
    env.reset()
    for s in steering:
        for a in acc_brake:
            env.step([s, a])
    env.close()


if __name__ == "__main__":
    test_nan_speed()
