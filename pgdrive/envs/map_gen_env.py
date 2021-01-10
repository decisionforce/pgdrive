import gym
import numpy as np

from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.pg_config import PgConfig
from pgdrive.pg_config.space import Box, Discrete
from pgdrive.scene_creator.blocks import Curve, Straight


def _get_gym_space_size(space):
    if isinstance(space, Box):
        return np.prod(space.shape)

    elif isinstance(space, Discrete):
        return 1

    else:
        raise NotImplementedError(type(space))


def _get_mapping_func(space, new_space_low=-1, new_space_high=+1):
    """
    Given a parameter space, return a function that can map a continuous action space [-1, 1]
    to the given parameter space.
    """
    if isinstance(space, Box):
        low = space.low[0]
        high = space.high[0]

        def f(act):
            act = np.asarray(act, dtype=np.float32)
            act = (act - new_space_low) / (new_space_high - new_space_low)  # Normalized to [0, 1]
            act = act * (high - low) + low
            return act

    elif isinstance(space, Discrete):
        n = space.n

        def f(act):
            # TODO this is tricky here. Should we use a categorical distribution or simply using a
            #  normal distribution to describe the Discrete parameter space?
            assert act.size == 1
            act = act.reshape(-1)[0]
            act = (act - new_space_low) / (new_space_high - new_space_low)  # Normalized to [0, 1]
            act = int(min(round(act * (n - 1)), n - 1))
            return act

    else:
        raise NotImplementedError(type(space))

    return f


def parameter_space_to_gym_space(para_space):
    assert hasattr(para_space, "spaces")
    spaces = para_space.spaces
    mappings = dict()
    ret_space_size = 0
    for name, space in spaces.items():
        func = _get_mapping_func(space)
        size = _get_gym_space_size(space)
        mappings[name] = (space, func, size, name)
        ret_space_size += size

    def transform(act):
        """
        Transform the raw action into a full dict of parameters.
        """
        act = np.asarray(act, dtype=np.float32)
        assert np.prod(act.shape) == ret_space_size
        count = 0
        para_dict = dict()
        for space, func, size, name in mappings.values():
            para = func(act[count: count + size])
            para_dict[name] = para
            count += size
        return para_dict

    ret_space = gym.spaces.Box(shape=(ret_space_size,), low=-1, high=1, dtype=np.float32)
    return ret_space, mappings, transform


class MapGenEnv(gym.Env):
    PGDRIVE_CONFIG = "pgdrive_config"

    @staticmethod
    def default_config() -> PgConfig:
        env_config = dict(pgdrive_config=dict(map=7))
        config = PgConfig(env_config)
        return config

    def __init__(self, config: dict = None):
        self.config = self.default_config()
        if config:
            self.config.update(config)

        self.env = None
        self.block_parameter_length = 10

        curve_space, curve_space_mappings, curve_transform = parameter_space_to_gym_space(Curve.PARAMETER_SPACE)
        straight_space, straight_space_mappings, straight_transform = \
            parameter_space_to_gym_space(Straight.PARAMETER_SPACE)

        self.action_space = gym.spaces.Dict(
            curve=curve_space,
            straight=straight_space,
            # gym.spaces.Box(low=-1.0, high=1.0, shape=(self.block_parameter_length,))
        )
        self.observation_space = None

    def step(self, action):

        # Step 1: Parse input action to structural data
        block_info = self._parse_action(action)

        # Step 2: Create / append a road block
        create_info = self._create_block(block_info)

        # Step 3: Get the return for the map-generation agent
        obs = self._get_obs()
        reward = self._get_reward(block_info, create_info)
        done = self._get_done(block_info, create_info)
        info = self._get_info(block_info, create_info)

        return obs, reward, done, info

    def render(self, mode='human'):
        # TODO How should we render the map generation process?
        pass

    def reset(self):
        if self.env is None:
            self.env = PGDriveEnv(self.config[self.PGDRIVE_CONFIG])
            self.env.reset()

        return self._get_obs()

    def close(self):
        if self.env:
            self.env.close()

    def _parse_action(self, action):
        block_info = None
        return block_info

    def _create_block(self, block_info):
        create_result = None
        return create_result

    def _get_obs(self):
        assert self.env is not None
        assert self.env.initialized

        import matplotlib.pyplot as plt
        plt.imshow(self.env.get_map(resolution=(512, 512)))
        plt.show()

        obs = self.env.get_map(resolution=(64, 64))

        plt.imshow(obs)
        plt.show()

        return obs

    def _get_reward(self, info):
        return 0.0

    def _get_done(self, info):
        return False

    def _get_info(self, info, create_info):
        return dict()
