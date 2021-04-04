import gym
import numpy as np

from pgdrive.envs.pgdrive_env_v2 import PGDriveEnvV2
from pgdrive.obs import LidarStateObservation
from pgdrive.obs.observation_type import ObservationType
from pgdrive.utils import PGConfig
from pgdrive.utils.math_utils import clip

DISTANCE = 50


class MinimalObservation(LidarStateObservation):
    _traffic_vehicle_state_dim = 15
    _traffic_vehicle_state_dim_wo_extra = 4

    def __init__(self, config):
        super(MinimalObservation, self).__init__(vehicle_config=config)
        self._state_obs_dim = list(self.state_obs.observation_space.shape)[0]

    @property
    def observation_space(self):
        shape = list(self.state_obs.observation_space.shape)
        if self.config["use_extra_state"]:
            shape[0] = shape[0] + self.config["lidar"]["num_others"] * self._traffic_vehicle_state_dim
        else:
            shape[0] = shape[0] + self.config["lidar"]["num_others"] * self._traffic_vehicle_state_dim_wo_extra
        return gym.spaces.Box(-0.0, 1.0, shape=tuple(shape), dtype=np.float32)

    def observe(self, vehicle):
        state = self.state_obs.observe(vehicle)
        other_v_info = []
        if self.config["lidar"]["num_others"] > 0:
            other_v_info += self.overwritten_get_surrounding_vehicles_info(
                lidar=vehicle.lidar,
                ego_vehicle=vehicle,
                num_others=self.config["lidar"]["num_others"]
            )
        return np.concatenate((state, np.asarray(other_v_info)))

    def overwritten_get_surrounding_vehicles_info(self, lidar, ego_vehicle, num_others: int = 4):
        from pgdrive.utils.math_utils import norm, clip
        surrounding_vehicles = list(lidar.get_surrounding_vehicles())
        surrounding_vehicles.sort(
            key=lambda v: norm(ego_vehicle.position[0] - v.position[0], ego_vehicle.position[1] - v.position[1])
        )
        surrounding_vehicles += [None] * num_others
        res = []
        for vehicle in surrounding_vehicles[:num_others]:
            if vehicle is not None:
                relative_position = ego_vehicle.projection(vehicle.position - ego_vehicle.position)
                res.append(clip((relative_position[0] / lidar.perceive_distance + 1) / 2, 0.0, 1.0))
                res.append(clip((relative_position[1] / lidar.perceive_distance + 1) / 2, 0.0, 1.0))
                relative_velocity = ego_vehicle.projection(vehicle.velocity - ego_vehicle.velocity)
                res.append(clip((relative_velocity[0] / ego_vehicle.max_speed + 1) / 2, 0.0, 1.0))
                res.append(clip((relative_velocity[1] / ego_vehicle.max_speed + 1) / 2, 0.0, 1.0))

                if self.config["use_extra_state"]:
                    res.extend(self.traffic_vehicle_state(vehicle))
            else:
                if self.config["use_extra_state"]:
                    res += [0.0] * self._traffic_vehicle_state_dim
                else:
                    res += [0.0] * self._traffic_vehicle_state_dim_wo_extra
        return res

    def traffic_vehicle_state(self, vehicle):
        s = []
        state = vehicle.to_dict()
        s.append(state['vx'] / vehicle.MAX_SPEED)
        s.append(state['vy'] / vehicle.MAX_SPEED)
        s.append(state["cos_h"])
        s.append(state["sin_h"])
        s.append(state["cos_d"])
        s.append(state["sin_d"])
        s.append(vehicle.target_speed / vehicle.MAX_SPEED)
        s.append(np.cos(vehicle.heading))
        s.append(np.sin(vehicle.heading))
        s.append(vehicle.action["steering"])
        s.append(vehicle.action["acceleration"])
        s = [self._to_zero_and_one(v) for v in s]
        return s

    @staticmethod
    def _to_zero_and_one(v):
        return (clip(v, -1, +1) + 1) / 2


class PGDriveEnvV2Minimal(PGDriveEnvV2):
    @classmethod
    def default_config(cls) -> PGConfig:
        config = super(PGDriveEnvV2Minimal, cls).default_config()
        config.update({"num_others": 4})
        config["vehicle_config"]["side_detector"]["num_lasers"] = 0
        config["vehicle_config"]["lane_line_detector"]["num_lasers"] = 0
        return config

    def get_single_observation(self, vehicle_config: "PGConfig") -> "ObservationType":
        return MinimalObservation(vehicle_config)

    def _post_process_config(self, config):
        config = super(PGDriveEnvV2Minimal, self)._post_process_config(config)
        assert config["vehicle_config"]["lidar"]["num_others"] == 0
        assert config["vehicle_config"]["lidar"]["num_lasers"] == 120
        config["vehicle_config"]["lidar"]["num_others"] = config["num_others"]
        return config


if __name__ == '__main__':

    def _act(env, action):
        assert env.action_space.contains(action)
        obs, reward, done, info = env.step(action)
        assert env.observation_space.contains(obs)
        assert np.isscalar(reward)
        assert isinstance(info, dict)


    env = PGDriveEnvV2Minimal({"num_others": 10, "map": "SSS"})
    try:
        obs = env.reset()
        assert env.observation_space.contains(obs)
        _act(env, env.action_space.sample())
        env.reset()
        for _ in range(100):
            _act(env, [0, 1])
    finally:
        env.close()
