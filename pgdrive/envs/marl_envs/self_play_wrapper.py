from pgdrive.envs.multi_agent_pgdrive import MultiAgentPGDrive


def get_self_play_env(env_class):
    env_name = env_class.__name__
    new_env_name = "SP{}".format(env_name)
    assert issubclass(env_class, MultiAgentPGDrive)

    class Wrapper(env_class):
        @classmethod
        def default_config(cls):
            config = super(Wrapper, cls).default_config()
            config["self_play_ratio"] = 0.0
            return config

        def __init__(self, config=None):
            if config.get("self_play_ratio", 0.0) > 0:
                num_agents = config.get("num_agents", self.default_config()["num_agents"])
                self._self_play_num_agents = int(config.get("self_play_ratio") * num_agents)
            else:
                self._self_play_num_agents = 0
            super(Wrapper, self).__init__(config)
            self._self_play_agents = set()

        def step(self, actions):
            # Compute the self-play agents action
            pass

            o, r, d, i = super(Wrapper, self).step(actions)

            # Filter out self-play agents
            pass

            return o, r, d, i

        def reset(self):
            obs = super(Wrapper, self).reset()

            # Filter out self-play agents
            pass

            return obs
