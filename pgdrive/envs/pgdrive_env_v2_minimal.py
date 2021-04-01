import gym
import numpy as np
from pgdrive.envs.pgdrive_env_v2 import PGDriveEnvV2
from pgdrive.obs.observation_type import LidarStateObservation, ObservationType
from pgdrive.utils import PGConfig


class MinimalObservation(LidarStateObservation):
    def __init__(self, vehicle_config):
        super(MinimalObservation, self).__init__(vehicle_config)

        if vehicle_config["state_set"] == 0:
            self.remove_indices = {}
        elif vehicle_config["state_set"] == 1:
            self.remove_indices = {
                # last actions
                5,
                6,
                # whether centering laterally
                8,
                # 9, 10, 14, 15 are the relative position to the checkpoints
                # The following are bend radius, dir, angle and so on.
                11,
                12,
                13,
                16,
                17,
                18
            }
        elif vehicle_config["state_set"] == 2:
            self.remove_indices = {
                # last actions
                5,
                6,
                # whether centering laterally
                8,
                # 9, 10, 14, 15 are the relative position to the checkpoints
                # The following are bend radius, dir, angle and so on.
                11,
                12,
                13,
                14,  # add
                15,  # add
                16,
                17,
                18,
            }
        elif vehicle_config["state_set"] == 3:  # keep last action
            self.remove_indices = {
                # last actions
                # 5,
                # 6,
                # whether centering laterally
                8,
                # 9, 10, 14, 15 are the relative position to the checkpoints
                # The following are bend radius, dir, angle and so on.
                11,
                12,
                13,
                16,
                17,
                18,
            }
        elif vehicle_config["state_set"] == 4:  # keep last action and centering
            self.remove_indices = {
                # last actions
                # 5,
                # 6,
                # whether centering laterally
                # 8,
                # 9, 10, 14, 15 are the relative position to the checkpoints
                # The following are bend radius, dir, angle and so on.
                11,
                12,
                13,
                16,
                17,
                18,
            }
        elif vehicle_config["state_set"] == 5:
            self.remove_indices = {
                # last actions
                # 5,
                # 6,
                # whether centering laterally
                # 8,
                # 9, 10, 14, 15 are the relative position to the checkpoints
                # The following are bend radius, dir, angle and so on.
                # 11,
                # 12,
                # 13,
                14,
                15,
                16,
                17,
                18,
            }

    @property
    def observation_space(self):
        shape = list(self.state_obs.observation_space.shape)
        shape[0] += self.config["lidar"]["num_others"] * 4
        shape[0] -= len(self.remove_indices)
        return gym.spaces.Box(-0.0, 1.0, shape=tuple(shape), dtype=np.float32)

    def observe(self, vehicle):
        """
        State observation + Navi info + 4 * closest vehicle info + Lidar points ,
        Definition of State Observation and Navi information can be found in **class StateObservation**
        Other vehicles' info: [
                              Projection of distance between ego and another vehicle on ego vehicle's heading direction,
                              Projection of distance between ego and another vehicle on ego vehicle's side direction,
                              Projection of speed between ego and another vehicle on ego vehicle's heading direction,
                              Projection of speed between ego and another vehicle on ego vehicle's side direction,
                              ] * 4, dim = 16

        Lidar points: 240 lidar points surrounding vehicle, starting from the vehicle head in clockwise direction

        :param vehicle: BaseVehicle
        :return: observation in 9 + 10 + 16 + 240 dim
        """
        state = self.state_obs.observe(vehicle)

        new_state = []

        for i, value in enumerate(state):
            if i in self.remove_indices:
                continue
            new_state.append(value)

        other_v_info = []
        assert vehicle.lidar is not None
        assert self.config["lidar"]["num_others"] > 0
        other_v_info += vehicle.lidar.get_surrounding_vehicles_info(vehicle, self.config["lidar"]["num_others"])
        # other_v_info += vehicle.lidar.get_cloud_points()  # Don't get the cloud point!
        return np.concatenate((new_state, np.asarray(other_v_info)))


class PGDriveEnvV2Minimal(PGDriveEnvV2):
    @classmethod
    def default_config(cls) -> PGConfig:
        config = super(PGDriveEnvV2Minimal, cls).default_config()
        config["vehicle_config"]["lidar"]["num_others"] = 4
        config["vehicle_config"]["state_set"] = 0
        return config

    def get_single_observation(self, vehicle_config: "PGConfig") -> "ObservationType":
        o = MinimalObservation(vehicle_config)
        return o


if __name__ == '__main__':

    def _act(env, action):
        assert env.action_space.contains(action)
        obs, reward, done, info = env.step(action)
        assert env.observation_space.contains(obs)
        assert np.isscalar(reward)
        assert isinstance(info, dict)

    env = PGDriveEnvV2Minimal()
    try:
        obs = env.reset()
        assert env.observation_space.contains(obs)
        _act(env, env.action_space.sample())
        for x in [-1, 0, 1]:
            env.reset()
            for y in [-1, 0, 1]:
                _act(env, [x, y])
    finally:
        env.close()
