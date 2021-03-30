from pgdrive.tests.test_envs.test_pgdrive_env import _act

from pgdrive.envs.pgdrive_env_v2 import PGDriveEnvV2


def test_pgdrive_env_v2():
    env = PGDriveEnvV2()
    assert env.observation_space.shape[0] == 120 + 19
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
    test_pgdrive_env_v2()
