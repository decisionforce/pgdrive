import numpy as np
import pytest

from pgdrive import PGDriveEnv

info_keys = []


def _act(env, action):
    assert env.action_space.contains(action)
    obs, reward, done, info = env.step(action)
    assert env.observation_space.contains(obs)
    assert np.isscalar(reward)
    assert isinstance(info, dict)
    for k in info_keys:
        assert k in info


key = "random_traffic"


@pytest.fixture(params=[True, False])
def config(request):
    yield dict(
        random_traffic=request.param
    )


def blackbox_test(config):
    env = PGDriveEnv(config=config)
    obs = env.reset()
    assert env.observation_space.contains(obs)

    _act(env, env.action_space.sample())
    for x in [-1, 0, 1]:
        env.reset()
        for y in [-1, 0, 1]:
            _act(env, [x, y])

    env.close()
    return


def test_func(config):
    print(config)


if __name__ == '__main__':
    pytest.main(["-s", "test_pgdrive_env.py"])
