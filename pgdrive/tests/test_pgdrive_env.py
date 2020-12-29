import os

import numpy as np
import pytest

from pgdrive import PGDriveEnv

# Key: case name, value: environmental config
blackbox_test_configs = dict(
    default=dict(),
    random_traffic=dict(random_traffic=True),
    large_seed=dict(start_seed=1000000),
    traffic_density_0=dict(traffic_density=0),
    traffic_density_1=dict(traffic_density=1),
    decision_repeat_50=dict(decision_repeat=50),
    map_7=dict(map=7),
    map_30=dict(map=30),
    map_CCC=dict(map="CCC"),
    envs_100=dict(environment_num=100),
    envs_1000=dict(environment_num=1000),
    envs_10000=dict(environment_num=10000),
    envs_100000=dict(environment_num=100000),
)

info_keys = ["cost", "velocity", "steering", "acceleration", "step_reward", "crash", "out_of_road", "arrive_dest"]

assert "__init__.py" not in os.listdir(os.path.dirname(__file__)), "Please remove __init__.py in tests directory."


def _act(env, action):
    assert env.action_space.contains(action)
    obs, reward, done, info = env.step(action)
    assert env.observation_space.contains(obs)
    assert np.isscalar(reward)
    assert isinstance(info, dict)
    for k in info_keys:
        assert k in info


@pytest.mark.parametrize("config", list(blackbox_test_configs.values()), ids=list(blackbox_test_configs.keys()))
def test_pgdrive_env_blackbox(config):
    env = PGDriveEnv(config=config)
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
    pytest.main(["-s", "test_pgdrive_env.py"])
