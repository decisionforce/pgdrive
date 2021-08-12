from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import setup_logger


class TestEnv(PGDriveEnv):
    def __init__(self):
        """
        TODO a small bug exists in scene 9 (30 blocks), traffic density > 0, respawn mode
        """
        super(TestEnv, self).__init__(
            {"map": "C", "traffic_density": 1.0, "environment_num": 10, "use_render": False}
        )


if __name__ == "__main__":
    setup_logger(True)
    env = TestEnv()
    import time
    start = time.time()
    o = env.reset()
    print(len(env.engine.traffic_manager._traffic_vehicles))
    for s in range(1, 100000):
        o, r, d, info = env.step([.0, 1.])
        print(s)
        # info["fuel"] = env.vehicle.energy_consumption
        # # env.render(text={"heading_diff": env.vehicle.heading_diff(env.vehicle.lane)})
        # assert env.observation_space.contains(o)
        # if (s + 1) % 100 == 0:
        #     print(
        #         "Finish {}/10000 simulation steps. Time elapse: {:.4f}. Average FPS: {:.4f}".format(
        #             s + 1,
        #             time.time() - start, (s + 1) / (time.time() - start)
        #         )
        #     )
    env.close()
