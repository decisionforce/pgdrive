from pgdrive.envs.top_down_env import TopDownPGDriveEnv
from pgdrive.scene_creator.ego_vehicle.base_vehicle import BaseVehicle
from pgdrive.world.top_down_observation.top_down_multi_channel import TopDownMultiChannel


class TopDownMultiChannelPGDriveEnv(TopDownPGDriveEnv):
    def initialize_observation(self):
        vehicle_config = BaseVehicle.get_vehicle_config(self.config["vehicle_config"])
        return TopDownMultiChannel(vehicle_config, self, self.config["rgb_clip"])


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    env = TopDownMultiChannelPGDriveEnv(dict(environment_num=1, map="XTO", traffic_density=0.1))
    env.reset()
    names = [
        "road_network", "navigation", "target_vehicle", "past_pos", "traffic t-4", "traffic t-3", "traffic t-2",
        "traffic t-1", "traffic t"
    ]

    for _ in range(10):
        o, *_ = env.step([0, 1])
        # env.reset()
    for _ in range(10):
        o, *_ = env.step([-0.05, 1])
    for _ in range(200):
        o, *_ = env.step([1, 1])
        fig, axes = plt.subplots(1, o.shape[-1], figsize=(12, 10))
        for o_i in range(o.shape[-1]):
            axes[o_i].imshow(o[..., o_i])
            axes[o_i].set_title(names[o_i])
        fig.suptitle("Multi-channel Top-down Observation")
        plt.show()
        print(o.mean())
    env.close()
