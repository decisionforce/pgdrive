from pgdrive.envs.pgdrive_env import PGDriveEnv
from drivingforce.generation_envs.test_pgdrive_env import _act


def test_pgdrive_env_rgb():
    env = PGDriveEnv(dict(offscreen_render=True))
    try:
        obs = env.reset()
        assert env.observation_space.contains(obs)
        _act(env, env.action_space.sample())
        for x in [-1, 0, 1]:
            env.reset()
            for y in [-1, 0, 1]:
                _act(env, [x, y])
    finally:
        env.close()


if __name__ == '__main__':
    test_pgdrive_env_rgb()
