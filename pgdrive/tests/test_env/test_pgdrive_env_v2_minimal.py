import numpy as np
from pgdrive.envs.pgdrive_env_v2_minimal import PGDriveEnvV2Minimal


def test_pgdrive_env_v2_minimal():
    def _act(env, action):
        assert env.action_space.contains(action)
        obs, reward, done, info = env.step(action)
        assert env.observation_space.contains(obs)
        assert np.isscalar(reward)
        assert isinstance(info, dict)

    def _test(env):
        try:
            obs = env.reset()
            assert env.observation_space.contains(obs)
            _act(env, env.action_space.sample())
            env.reset()
            for _ in range(100):
                _act(env, [0, 1])
        finally:
            env.close()

    _test(PGDriveEnvV2Minimal({"num_others": 4, "use_extra_state": True, "traffic_density": 0.5}))
    _test(PGDriveEnvV2Minimal({"num_others": 0, "use_extra_state": True, "traffic_density": 0.5}))
    _test(PGDriveEnvV2Minimal({"num_others": 4, "use_extra_state": False, "traffic_density": 0.5}))
    _test(PGDriveEnvV2Minimal({"num_others": 0, "use_extra_state": False, "traffic_density": 0.5}))


if __name__ == '__main__':
    test_pgdrive_env_v2_minimal()
