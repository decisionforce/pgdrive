import time

from pgdrive.envs.pgdrive_env import PGDriveEnv

if __name__ == "__main__":
    env = PGDriveEnv(
        dict(
            use_render=True,
            manual_control=True,
            pg_world_config=dict(_use_tiny=True)
        )
    )

    start = time.time()
    env.reset()
    env.render()
    print("Render cost time: ", time.time() - start)
    while True:
        o, r, d, info = env.step([0, 1])
        env.render()
    env.close()
