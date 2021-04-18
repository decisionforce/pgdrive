from pgdrive.envs.pgdrive_env_v2 import PGDriveEnvV2


def check_render_can_close_and_restart():
    try:
        for m in ["X", "O", "C", "S", "R", "r", "T"]:
            env = PGDriveEnvV2({"map": m, "use_render": True, "fast": True})
            o = env.reset()
            for _ in range(300):
                assert env.observation_space.contains(o)
                o, r, d, i = env.step([1, 1])
                if d:
                    break
            env.close()
    finally:
        if "env" in locals():
            env = locals()["env"]
            env.close()


if __name__ == '__main__':
    check_render_can_close_and_restart()
