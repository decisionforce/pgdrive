from gym.spaces import Box

from pg_drive.envs.generalization_racing import GeneralizationRacing
from pg_drive.pg_config.pg_config import PgConfig


class ActionRepeat(GeneralizationRacing):
    @staticmethod
    def default_config() -> PgConfig:
        config = GeneralizationRacing.default_config()
        config.add("max_action_repeat", 100)
        config.add("min_action_repeat", 1)
        config.add("gamma", 1.0)
        return config

    def __init__(self, config: dict = None):
        super(ActionRepeat, self).__init__(config)
        self.action_space = Box(
            shape=(self.action_space.shape[0] + 1, ),
            high=self.action_space.high[0],
            low=self.action_space.low[0],
            dtype=self.action_space.dtype
        )
        self.low = self.action_space.low[0]
        self.high = self.action_space.high[0]
        self.action_repeat_low = self.config["min_action_repeat"]
        self.action_repeat_high = self.config["max_action_repeat"]
        assert self.action_repeat_low > 0

    def step(self, action, render=False, **render_kwargs):
        action_repeat = action[-1]
        action_repeat = round(
            (action_repeat - self.low) / (self.high - self.low) * (self.action_repeat_high - self.action_repeat_low) +
            self.action_repeat_low
        )
        assert action_repeat > 0

        ret = []
        render_list = []
        real_ret = 0.0
        for repeat in range(action_repeat):
            o, r, d, i = super(ActionRepeat, self).step(action)
            if render:
                render_list.append(self.render(**render_kwargs))
            ret.append(r)
            real_ret += r
            if d:
                break

        discounted = 0.0
        for r in reversed(ret):
            discounted = self.config["gamma"] * discounted + r

        i["action_repeat"] = repeat + 1
        i["real_return"] = real_ret
        i["render"] = render_list

        return o, discounted, d, i


if __name__ == '__main__':
    env = ActionRepeat()
    env.reset()
    env.step(env.action_space.sample())
    env.close()
