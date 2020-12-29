import pytest
from pgdrive import ActionRepeat


def _test_action_repeat(config):
    env = ActionRepeat(config)
    env.reset()
    for i in range(100):
        _, _, d, _ = env.step(env.action_space.sample())
        if d:
            env.reset()
    env.close()


def test_action_repeat_max_ar():
    _test_action_repeat(dict(max_action_repeat=5))
    _test_action_repeat(dict(max_action_repeat=1))


def test_action_repeat_fixed_ar():
    _test_action_repeat(dict(fixed_action_repeat=5))
    _test_action_repeat(dict(fixed_action_repeat=1))


if __name__ == '__main__':
    pytest.main(["-sv", "test_action_repeat_env.py"])
