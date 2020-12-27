import gym
from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.pg_config import PgConfig


class MapGenEnv(gym.Env):
    PGDRIVE_CONFIG = "pgdrive_config"

    @staticmethod
    def default_config() -> PgConfig:
        env_config = dict(
            pgdrive_config=dict(map=7)
        )
        config = PgConfig(env_config)
        return config

    def __init__(self, config: dict = None):
        self.config = self.default_config()
        if config:
            self.config.update(config)

        self.env = None
        self.action_space = None
        self.observation_space = None

    def step(self, action):

        # Step 1: Parse input action to structural data
        block_info = self._parse_action(action)

        # Step 2: Create / append a road block
        create_result = self._create_block(block_info)

        # Step 3: Get the return for the map-generation agent
        obs = self._get_obs()
        reward = self._get_reward(create_result)
        done = self._get_done(create_result)
        info = self._get_info(block_info, create_result)

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
        self.env.close()

    def _parse_action(self, action):
        block_info = None
        return block_info

    def _create_block(self, block_info):
        create_result = None
        return create_result

    def _get_obs(self):
        obs = None
        return obs

    def _get_reward(self, info):
        return 0.0

    def _get_done(self, info):
        return False

    def _get_info(self, info):
        return dict()
