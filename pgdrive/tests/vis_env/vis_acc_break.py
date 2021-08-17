from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import setup_logger


class TestEnv(PGDriveEnv):
    def __init__(self):
        super(TestEnv, self).__init__(
            {
                "environment_num": 5,
                "traffic_density": .0,
                "use_render": True,
                "fast": True,
                "map": "SSS",
                "vehicle_config": {
                    "enable_reverse": True,
                    # "random_color": True
                    # "show_lidar": True,
                    # "spawn_lane_index":("1r1_0_", "1r1_1_", 0),
                    # "destination_node":"2R1_3_",
                    # "show_side_detector": True,
                    # "show_lane_line_detector": True,
                    # "side_detector": dict(num_lasers=2, distance=50),
                    # "lane_line_detector": dict(num_lasers=2, distance=50),
                    # # "show_line_to_dest": True,
                    # "show_dest_mark": True
                }
            }
        )


if __name__ == "__main__":
    setup_logger(True)
    env = TestEnv()
    import time
    start = time.time()
    o = env.reset()

    for s in range(1, 100000):
        o, r, d, info = env.step([.0, 1.])
        # info["fuel"] = env.vehicle.energy_consumption
        env.render(text={"heading_diff": env.vehicle.heading_diff(env.vehicle.lane)})
        # assert env.observation_space.contains(o)
        # if (s + 1) % 100 == 0:
        #     print(
        #         "Finish {}/10000 simulation steps. Time elapse: {:.4f}. Average FPS: {:.4f}".format(
        #             s + 1,f
        #             time.time() - start, (s + 1) / (time.time() - start)
        #         )
        #     )
        if d:
            # env.close()
            print(len(env.engine._spawned_objects))
            env.reset()
