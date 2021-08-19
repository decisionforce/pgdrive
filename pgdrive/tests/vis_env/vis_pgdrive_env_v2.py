from pgdrive import PGDriveEnv
from pgdrive.utils import setup_logger

if __name__ == "__main__":
    setup_logger(True)
    env = PGDriveEnv(
        {
            "start_seed": 0,
            "environment_num": 10,
            "fast": True,
            "use_render": True,
            "manual_control": True,
            # "vehicle_config": {
            #     "show_side_detector": True,
            #     "show_lane_line_detector": True,
            #     "show_navi_mark": True,
            #     "show_lidar": True,
            # }
            "random_agent_model": True,
            "random_lane_width": True,
            "load_map_from_json": False
        }
    )

    o = env.reset()
    print("vehicle num", len(env.engine.traffic_manager.vehicles))
    for i in range(1, 100000):
        o, r, d, info = env.step([0, 1])
        info["fuel"] = env.vehicle.energy_consumption
        env.render(
            text={
                "left": env.vehicle.dist_to_left_side,
                "right": env.vehicle.dist_to_right_side,
                "width": env.vehicle.lane.width,
                "white_lane_line": env.vehicle.on_white_continuous_line,
                "reward": r,
            }
        )
        if d:
            print("Reset")
            env.reset()
    env.close()
