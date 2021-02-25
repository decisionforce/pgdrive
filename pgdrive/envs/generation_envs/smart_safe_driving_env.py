import gym
import numpy as np

from pgdrive.envs.generation_envs.safe_driving_env import SafeDrivingEnv
from pgdrive.examples.ppo_expert import expert


class SmartSafeDrivingEnv(SafeDrivingEnv):
    def __init__(self, config: dict = None):
        super(SmartSafeDrivingEnv, self).__init__(config)
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)

    def saver(self, action):
        steering = action[0]
        throttle = action[1]
        take_over = action[2] > 0.0 and self.config["use_saver"]  # In scale [-1, 1]

        if take_over:
            obs = self.observation.observe(self.vehicle)
            saver_a = expert(obs, deterministic=False)
            steering, throttle = saver_a

        pre_save = self.takeover
        self.takeover = True if take_over else False
        self.step_info["takeover_start"] = True if not pre_save and self.takeover else False
        self.step_info["takeover_end"] = True if pre_save and not self.takeover else False
        return steering, throttle


if __name__ == '__main__':
    e = SmartSafeDrivingEnv()
    print(e.action_space)
    print(e.observation_space)
    e.reset()
    for _ in range(100):
        i = e.step(e.action_space.sample())
    print(i)
    e.close()
