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
    for i in range(1, 100000000):
        o, r, d, info = env.step([0, 1])
        text = {"save": env.save_mode}
        env.render(text=text)
        if d:
            env.reset()
    env.close()
