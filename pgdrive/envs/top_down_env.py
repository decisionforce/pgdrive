from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.pg_config import PGConfig
from pgdrive.scene_creator.ego_vehicle.base_vehicle import BaseVehicle
from pgdrive.world.top_down_observation.top_down_observation import TopDownObservation


class TopDownPGDriveEnv(PGDriveEnv):
    @classmethod
    def default_config(cls) -> PGConfig:
        config = PGDriveEnv.default_config()
        config["vehicle_config"]["lidar"] = {"num_lasers": 0, "distance": 0}  # Remove lidar
        return config

    def initialize_observation(self):
        vehicle_config = BaseVehicle.get_vehicle_config(self.config["vehicle_config"])
        return TopDownObservation(vehicle_config, self, self.config["rgb_clip"])


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    frames = []
    env = TopDownPGDriveEnv(dict(environment_num=1, map="C", traffic_density=1.0))
    import numpy as np

    env.reset()
    for _ in range(10):
        o, *_ = env.step([0, 1])
        # env.reset()
    for _ in range(10):
        o, *_ = env.step([-0.05, 1])
        assert env.observation_space.contains(o)
    for _ in range(200):
        o, *_ = env.step([0.01, 1])

        frames.append(np.array(o) * 255)
        plt.imshow(o)
        plt.show()
        print(o.mean())
    env.close()
