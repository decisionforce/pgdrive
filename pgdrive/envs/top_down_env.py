from pgdrive.envs.observation_type import TopDownObservation
from pgdrive.envs.pgdrive_env import PGDriveEnv

from pgdrive.pg_config import PGConfig
from pgdrive.scene_creator.ego_vehicle.base_vehicle import BaseVehicle


class TopDownPGDriveEnv(PGDriveEnv):
    @staticmethod
    def default_config() -> PGConfig:
        config = PGDriveEnv.default_config()
        config["use_topdown"] = True
        return config

    def initialize_observation(self):
        vehicle_config = BaseVehicle.get_vehicle_config(self.config["vehicle_config"])
        return TopDownObservation(vehicle_config, self, self.config["rgb_clip"])


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    env = TopDownPGDriveEnv(dict(environment_num=100))
    # env.reset()
    for _ in range(10):
        env.reset()
        o, *_ = env.step(env.action_space.sample())
        plt.imshow(o)
        plt.show()
        print(o)
    env.close()
