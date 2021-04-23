import numpy as np

from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import PGConfig, get_np_random


class ChangeFrictionEnv(PGDriveEnv):
    @staticmethod
    def default_config() -> PGConfig:
        config = PGDriveEnv.default_config()
        config.update(
            {
                "change_friction": True,
                "friction_min": 0.8,
                "friction_max": 1.2,
                "vehicle_config": {
                    "wheel_friction": 1.0
                }
            }
        )
        return config

    def __init__(self, config=None):
        super(ChangeFrictionEnv, self).__init__(config)
        self.parameter_list = dict()
        self._random_state = get_np_random(0)  # Use a fixed seed.
        for k in self.maps:
            self.parameter_list[k] = dict(
                wheel_friction=self._random_state.uniform(self.config["friction_min"], self.config["friction_max"])
            )

    def _reset_agents(self):
        if self.config["change_friction"] and self.vehicle is not None:
            parameter = self.parameter_list[self.current_seed]
            self.vehicle.set_wheel_friction(parameter["wheel_friction"])
        super(ChangeFrictionEnv, self)._reset_agents()


if __name__ == '__main__':
    env = ChangeFrictionEnv(config={"environment_num": 100, "start_seed": 1000, "change_friction": True})
    env.seed(1010)
    obs = env.reset()
    for s in range(100):
        action = np.array([0.0, 1.0])
        o, r, d, i = env.step(action)
        if d:
            env.reset()
