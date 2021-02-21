from pgdrive.envs.generation_envs.side_pass_env import SidePassEnv

# setup_logger(True)

if __name__ == "__main__":
    env = SidePassEnv({"manual_control": True, "use_render": True})
    o = env.reset()
    for i in range(1, 100000):
        o, r, d, info = env.step([0, 1])
        env.render()
        if d:
            env.reset()
    env.close()
