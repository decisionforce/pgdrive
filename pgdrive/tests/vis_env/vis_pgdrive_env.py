from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import setup_logger


class TestEnv(PGDriveEnv):
    def __init__(self):
        """
        TODO a small bug exists in scene 9 (30 blocks), traffic density > 0, respawn mode
        """
        super(TestEnv, self).__init__(
            {
                "environment_num": 10,
                "traffic_density": 1,
                "traffic_mode": "respawn",
                "start_seed": 12,
                "onscreen_message": True,
                # "debug_physics_world": True,
                "pstats": True,
                "global_light": True,
                # "debug_static_world":True,
                "cull_scene": False,
                # "controller":"joystick",
                "manual_control": True,
                # "use_render": True,
                "decision_repeat": 5,
                "rgb_clip": True,
                "debug": True,
                "fast": True,
                # "map_config": {
                #     Map.GENERATE_TYPE: MapGenerateMethod.BIG_BLOCK_SEQUENCE,
                #     Map.GENERATE_CONFIG: "SXO",
                #     Map.LANE_WIDTH: 3.5,
                #     Map.LANE_NUM: 3,
                # },
                "map": "CCC",
                "driving_reward": 1.0,
                "vehicle_config": {
                    "enable_reverse": True,
                    # "show_lidar": True,
                    # "show_side_detector": True,
                    # "show_lane_line_detector": True,
                    "side_detector": dict(num_lasers=2, distance=50),
                    "lane_line_detector": dict(num_lasers=2, distance=50),
                    # "show_line_to_dest": True,
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
        o, r, d, info = env.step([1.0, 0.])
        info["fuel"] = env.vehicle.energy_consumption
        # env.render(text={"heading_diff": env.vehicle.heading_diff(env.vehicle.lane)})
        assert env.observation_space.contains(o)
        # env.render(
        #     text={
        #         "reward": r,
        #         "lane_index": env.vehicle.lane_index,
        #         "dist_to_left": env.vehicle.dist_to_left_side,
        #         "dist_to_right": env.vehicle.dist_to_right_side,
        #         "out_of_route": env.vehicle.out_of_route,
        #         "current_seed": env.current_seed,
        #         "car_heading": env.vehicle.heading_theta,
        #         "lane_heading": env.vehicle.lane.heading_at(0)
        #     }
        # )
        # env.render(text={"map":env.current_seed})
        # if d:
        #     print("Reset")
        #     env.reset()
        if (s + 1) % 100 == 0:
            print(
                "Finish {}/10000 simulation steps. Time elapse: {:.4f}. Average FPS: {:.4f}".format(
                    s + 1,
                    time.time() - start, (s + 1) / (time.time() - start)
                )
            )
    env.close()
