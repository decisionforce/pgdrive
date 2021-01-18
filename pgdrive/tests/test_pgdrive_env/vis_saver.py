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
        text = {"save": env.save_mode, "max_action":(max_s, max_t)}
        env.render(text=text)
        if d:
            env.reset()
    env.close()
