from pgdrive.envs.generation_envs.safe_driving_env import SafeDrivingEnv

if __name__ == "__main__":
    env = SafeDrivingEnv(
        {
            # "environment_num": 1,
            # "traffic_density": 0.,
            # "traffic_mode": "reborn",
            # "start_seed": 5,
            # "controller": "joystick",
            "manual_control": True,
            "use_render": True,
            "out_of_road_constrain":False,
            "crash_constrain":False,
            # "save_level":0.3,
            # "takeover_penalty":0.5
            # "use_saver":False,
            # "map": "Crrrrrrrrrr"
        }
    )

    o = env.reset()
    max_s = 0
    max_t = 0
    for i in range(1, 100000000):
        o, r, d, info = env.step([0, 1])
        max_s = max(max_s, info["raw_action"][0])
        max_t = max(max_t, info["raw_action"][1])
        text = {"save": env.save_mode, "max_action":(max_s, max_t), "raw_reward":env.step_info["raw_step_reward"],
                "reward":r, "speed":env.vehicle.speed, "high_speed":info["high_speed"]}
        env.render(text=text)
        if d:
            print(info)
            env.reset()
    env.close()
