from pgdrive.envs.generation_envs.safe_driving_env import SafeDrivingEnv

if __name__ == "__main__":
    env = SafeDrivingEnv(
        {
            "environment_num": 1,
            "traffic_density": 0.3,
            "traffic_mode": "reborn",
            "start_seed": 5,
            # "controller": "joystick",
            "manual_control": True,
            "use_render": True,
            "map": "CRCRCRCR"
        }
    )

    o = env.reset()
    for i in range(1, 100000000):
        o, r, d, info = env.step([0, 1])
        # print(env.vehicle.lane_index)
        # i_1 = env.vehicle.routing_localization.target_checkpoints_index[0]
        # i_2 = env.vehicle.routing_localization.target_checkpoints_index[1]
        # print("target:", env.vehicle.routing_localization.checkpoints[i_1])
        text = {"save": env.save_mode}
        env.render(text=text)
        if info["arrive_dest"]:
            env.reset()
    env.close()
