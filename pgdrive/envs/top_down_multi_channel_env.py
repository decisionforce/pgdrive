from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.scene_creator.ego_vehicle.base_vehicle import BaseVehicle
from pgdrive.world.top_down_observation.top_down_multi_channel import TopDownMultiChannel


class TopDownMultiChannelPGDriveEnv(PGDriveEnv):
    def initialize_observation(self):
        vehicle_config = BaseVehicle.get_vehicle_config(self.config["vehicle_config"])
        return TopDownMultiChannel(vehicle_config, self, self.config["rgb_clip"])


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    env = TopDownMultiChannelPGDriveEnv(dict(environment_num=1, map="C", traffic_density=1.0))
    env.reset()
    for _ in range(10):
        o, *_ = env.step([0, 1])
        # env.reset()
    for _ in range(10):
        o, *_ = env.step([-0.05, 1])
    for _ in range(200):
        o, *_ = env.step([0.01, 1])
        fig, axes = plt.subplots(1, o.shape[-1])
        for o_i in range(o.shape[-1]):
            axes[o_i].imshow(o[..., o_i])
            axes[o_i].set_title(TopDownMultiChannel.CHANNEL_NAMES[o_i])
        fig.suptitle("Multi-channel Top-down Observation")
        plt.show()
        print(o.mean())
    env.close()
