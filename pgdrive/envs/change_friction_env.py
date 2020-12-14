import numpy as np
from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.pg_config.pg_config import PgConfig


class ChangeFrictionEnv(PGDriveEnv):
    @staticmethod
    def default_config() -> PgConfig:
        config = PGDriveEnv.default_config()
        config.add("change_friction", True)
        config.add("friction_min", 0.6)
        config.add("friction_max", 1.2)
        return config

    def __init__(self, config=None):
        super(ChangeFrictionEnv, self).__init__(config)
        self.parameter_list = dict()
        for k in self.maps:
            self.parameter_list[k] = dict(
                wheel_friction=np.random.uniform(self.config["friction_min"], self.config["friction_max"])
            )

    def reset(self):
        index = np.random.choice(list(self.parameter_list.keys()))
        parameter = self.parameter_list[index]
        if self.vehicle is None or (not self.config["change_friction"]):
            pass
        else:
            self.vehicle.vehicle_config["wheel_friction"] = parameter["wheel_friction"]
            print("The friction is changed to: ", parameter["wheel_friction"])
        ret = super(ChangeFrictionEnv, self).reset()
        return ret


if __name__ == '__main__':
    env = ChangeFrictionEnv(config={"environment_num": 100, "start_seed": 1000})
    env.seed(100000)
    obs = env.reset()
    for s in range(10000):
        action = np.array([0.0, 1.0])
        o, r, d, i = env.step(action)
        if d:
            env.reset()
