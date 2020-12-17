from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import setup_logger

setup_logger(debug=True)


class TestEnv(PGDriveEnv):
    def __init__(self):
        super(TestEnv, self).__init__(
            {
                "environment_num": 4,
                "traffic_density": 0.1,
                "manual_control": True,
                "vehicle_config": {
                    "lidar": (240, 50, 4)
                },
                "use_render": True,
            }
        )


if __name__ == "__main__":
    from pgdrive.scene_creator.ego_vehicle.vehicle_module.lidar import Lidar
    Lidar.enable_show = True
    env = TestEnv()
    env.reset()

    for i in range(1, 100000):
        o, r, d, info = env.step([0, 1])
        env.render("Test: {}".format(i))
    env.close()
