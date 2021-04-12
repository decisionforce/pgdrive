import numpy as np
from pgdrive.envs.marl_envs.marl_inout_roundabout import MultiAgentRoundaboutEnv


def _act(env, action):
    assert env.action_space.contains(action)
    obs, reward, done, info = env.step(action)
    assert env.observation_space.contains(obs)
    if env.num_agents > 1:
        assert isinstance(reward, dict)
    else:
        assert np.isscalar(reward)
    assert isinstance(info, dict)
    return obs, reward, done, info


def test_ma_roundabout_env():
    env = MultiAgentRoundaboutEnv({"num_agents": 1, "vehicle_config": {"lidar": {"num_others": 8}}})
    try:
        obs = env.reset()
        assert env.observation_space.contains(obs)
        for step in range(100):
            o, r, d, i = _act(env, [1, 1])
            if step == 0:
                assert not d
    finally:
        env.close()

    env = MultiAgentRoundaboutEnv({"num_agents": 1, "vehicle_config": {"lidar": {"num_others": 0}}})
    try:
        obs = env.reset()
        assert env.observation_space.contains(obs)
        for step in range(100):
            o, r, d, i = _act(env, [1, 1])
            if step == 0:
                assert not d
    finally:
        env.close()

    env = MultiAgentRoundaboutEnv({"num_agents": 4, "vehicle_config": {"lidar": {"num_others": 8}}})
    try:
        obs = env.reset()
        assert env.observation_space.contains(obs)
        for step in range(100):
            act = {k: [1, 1] for k in env.vehicles.keys()}
            o, r, d, i = _act(env, act)
            if step == 0:
                assert not any(d.values())
    finally:
        env.close()

    env = MultiAgentRoundaboutEnv({"num_agents": 4, "vehicle_config": {"lidar": {"num_others": 0}}})
    try:
        obs = env.reset()
        assert env.observation_space.contains(obs)
        for step in range(100):
            act = {k: [1, 1] for k in env.vehicles.keys()}
            o, r, d, i = _act(env, act)
            if step == 0:
                assert not any(d.values())
    finally:
        env.close()


def test_ma_roundabout_horizon():
    # test horizon
    env = MultiAgentRoundaboutEnv({"horizon": 100, "num_agents": 2, "vehicle_config": {"lidar": {"num_others": 2}}})
    try:
        obs = env.reset()
        assert env.observation_space.contains(obs)
        for step in range(1000):
            act = {k: [1, 1] for k in env.vehicles.keys()}
            o, r, d, i = _act(env, act)
            if step == 0:
                assert not any(d.values())
            if any(d.values()):
                print("Step {}, Done: {}".format(step, d))
            if d["__all__"]:
                break
    finally:
        env.close()


if __name__ == '__main__':
    # test_ma_roundabout_env()
    test_ma_roundabout_horizon()
