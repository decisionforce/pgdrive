from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import setup_logger


class TestEnv(PGDriveEnv):
    def __init__(self):
        super(TestEnv, self).__init__({"use_render": True, "manual_control": True, "environment_num": 100})


if __name__ == "__main__":
    setup_logger(True)
    env = TestEnv()

    o = env.reset()
    print("vehicle num", len(env.scene_manager.traffic_manager.vehicles))
    for i in range(1, 100000):
        o, r, d, info = env.step([0, 1])
        env.render(
            text={
                "vehicle_num": len(env.scene_manager.traffic_manager.traffic_vehicles),
                "dist_to_left:": env.vehicle.dist_to_left_side,
                "dist_to_right:": env.vehicle.dist_to_right_side
            }
        )
    env.close()
