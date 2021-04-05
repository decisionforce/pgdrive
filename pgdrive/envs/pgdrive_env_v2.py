import logging

import gym
import numpy as np
from pgdrive.constants import DEFAULT_AGENT
from pgdrive.envs.pgdrive_env import PGDriveEnv as PGDriveEnvV1
from pgdrive.obs import ImageStateObservation, ObservationType, LidarStateObservation
from pgdrive.scene_manager.traffic_manager import TrafficMode
from pgdrive.utils import PGConfig, clip


class LidarStateObservationV2(LidarStateObservation):
    def __init__(self, vehicle_config):
        super(LidarStateObservationV2, self).__init__(vehicle_config)

    @property
    def observation_space(self):
        shape = [21, ]
        if self.config["lidar"]["num_lasers"] > 0 and self.config["lidar"]["distance"] > 0:
            # Number of lidar rays and distance should be positive!
            shape[0] += self.config["lidar"]["num_lasers"] + self.config["lidar"]["num_others"] * 4
        return gym.spaces.Box(-0.0, 1.0, shape=tuple(shape), dtype=np.float32)

    def observe(self, vehicle):
        state = self.state_observe(vehicle)
        other_v_info = []
        if vehicle.lidar is not None:
            if self.config["lidar"]["num_others"] > 0:
                other_v_info += vehicle.lidar.get_surrounding_vehicles_info(vehicle,
                                                                            self.config["lidar"]["num_others"])
            other_v_info += self._add_noise_to_cloud_points(vehicle.lidar.get_cloud_points())
        return np.concatenate((state, np.asarray(other_v_info)))

    def _add_noise_to_cloud_points(self, points):
        return super(LidarStateObservationV2, self)._add_noise_to_cloud_points(points)

    def state_observe(self, vehicle):
        navi_info = vehicle.routing_localization.get_navi_info()
        navi_info = [navi_info[0], navi_info[1], navi_info[5], navi_info[6]]  # Only keep the checkpoints information!
        ego_state = self.vehicle_state(vehicle)
        return np.asarray(ego_state + navi_info, dtype=np.float32)

    def vehicle_state(self, vehicle):
        """
        Wrap vehicle states to list
        """
        # update out of road
        info = []
        if hasattr(vehicle, "side_detector") and vehicle.side_detector is not None:
            info += vehicle.side_detector.get_cloud_points()
        else:
            raise ValueError()

        # current_reference_lane = vehicle.routing_localization.current_ref_lanes[-1]
        info += [
            # vehicle.heading_diff(current_reference_lane),
            # Note: speed can be negative denoting free fall. This happen when emergency brake.
            clip((vehicle.speed + 1) / (vehicle.max_speed + 1), 0.0, 1.0),
            clip((vehicle.steering / vehicle.max_steering + 1) / 2, 0.0, 1.0),
            clip((vehicle.last_current_action[0][0] + 1) / 2, 0.0, 1.0),
            clip((vehicle.last_current_action[0][1] + 1) / 2, 0.0, 1.0)
        ]
        heading_dir_last = vehicle.last_heading_dir
        heading_dir_now = vehicle.heading
        cos_beta = heading_dir_now.dot(heading_dir_last
                                       ) / (np.linalg.norm(heading_dir_now) * np.linalg.norm(heading_dir_last))
        beta_diff = np.arccos(clip(cos_beta, 0.0, 1.0))
        # print(beta)
        yaw_rate = beta_diff / 0.1
        # print(yaw_rate)
        info.append(clip(yaw_rate, 0.0, 1.0))

        if vehicle.lane_line_detector is not None:
            info += vehicle.lane_line_detector.get_cloud_points()
        # else:
        #     _, lateral = vehicle.lane.local_coordinates(vehicle.position)
        #     info.append(
        #         clip((lateral * 2 / vehicle.routing_localization.get_current_lane_width() + 1.0) / 2.0, 0.0, 1.0)
        #     )
        return info


class PGDriveEnvV2(PGDriveEnvV1):
    DEFAULT_AGENT = DEFAULT_AGENT

    @staticmethod
    def default_config() -> PGConfig:
        config = PGDriveEnvV1.default_config()
        config.update(
            dict(
                # ===== Traffic =====
                traffic_density=0.1,
                traffic_mode=TrafficMode.Trigger,  # "reborn", "trigger", "hybrid"
                random_traffic=False,  # Traffic is randomized at default.

                # ===== Cost Scheme =====
                crash_vehicle_cost=1.,
                crash_object_cost=1.,
                out_of_road_cost=1.,

                # ===== Reward Scheme =====
                # See: https://github.com/decisionforce/pgdrive/issues/283
                success_reward=10.0,
                out_of_road_penalty=5.0,
                crash_vehicle_penalty=5.0,
                crash_object_penalty=5.0,
                acceleration_penalty=0.0,
                driving_reward=1.0,
                general_penalty=0.0,
                speed_reward=0.5,
                use_lateral=False,

                # See: https://github.com/decisionforce/pgdrive/issues/297
                vehicle_config=dict(
                    lidar=dict(num_lasers=120, distance=50, num_others=0, gaussian_noise=0.0, dropout_prob=0.0),
                    side_detector=dict(num_lasers=6, distance=50),  # laser num, distance
                    lane_line_detector=dict(num_lasers=6, distance=20),  # laser num, distance
                ),

                # Disable map loading!
                load_map_from_json=False,
                _load_map_from_json="",
            )
        )
        config.remove_keys([])
        return config

    def __init__(self, config: dict = None):
        super(PGDriveEnvV2, self).__init__(config=config)

    def done_function(self, vehicle_id: str):
        vehicle = self.vehicles[vehicle_id]
        done = False
        done_info = dict(crash_vehicle=False, crash_object=False, out_of_road=False, arrive_dest=False)
        if vehicle.arrive_destination:
            done = True
            logging.info("Episode ended! Reason: arrive_dest.")
            done_info["arrive_dest"] = True
        elif vehicle.crash_vehicle:
            done = True
            logging.info("Episode ended! Reason: crash. ")
            done_info["crash_vehicle"] = True
        # elif vehicle.out_of_route or not vehicle.on_lane or vehicle.crash_sidewalk:
        elif vehicle.on_yellow_continuous_line or (not vehicle.on_lane) or vehicle.crash_sidewalk:
            done = True
            logging.info("Episode ended! Reason: out_of_road.")
            done_info["out_of_road"] = True
        elif vehicle.crash_object:
            done = True
            done_info["crash_object"] = True

        # for compatibility
        # crash almost equals to crashing with vehicles
        done_info["crash"] = done_info["crash_vehicle"] or done_info["crash_object"]
        return done, done_info

    def cost_function(self, vehicle_id: str):
        vehicle = self.vehicles[vehicle_id]
        step_info = dict()
        step_info["cost"] = 0
        if vehicle.crash_vehicle:
            step_info["cost"] = self.config["crash_vehicle_cost"]
        elif vehicle.crash_object:
            step_info["cost"] = self.config["crash_object_cost"]
        elif vehicle.out_of_route:
            step_info["cost"] = self.config["out_of_road_cost"]
        return step_info['cost'], step_info

    def reward_function(self, vehicle_id: str):
        """
        Override this func to get a new reward function
        :param vehicle_id: id of BaseVehicle
        :return: reward
        """
        vehicle = self.vehicles[vehicle_id]
        step_info = dict()

        # Reward for moving forward in current lane
        current_lane = vehicle.lane if vehicle.lane in vehicle.routing_localization.current_ref_lanes else \
            vehicle.routing_localization.current_ref_lanes[0]
        long_last, _ = current_lane.local_coordinates(vehicle.last_position)
        long_now, lateral_now = current_lane.local_coordinates(vehicle.position)

        reward = 0.0

        # reward for lane keeping, without it vehicle can learn to overtake but fail to keep in lane
        if self.config["use_lateral"]:
            lateral_factor = clip(
                1 - 2 * abs(lateral_now) / vehicle.routing_localization.get_current_lane_width(), 0.0, 1.0
            )
        else:
            lateral_factor = 1.0

        reward += self.config["driving_reward"] * (long_now - long_last) * lateral_factor

        reward += self.config["speed_reward"] * (vehicle.speed / vehicle.max_speed)
        step_info["step_reward"] = reward

        if vehicle.crash_vehicle:
            reward = -self.config["crash_vehicle_penalty"]
        elif vehicle.crash_object:
            reward = -self.config["crash_object_penalty"]
        elif vehicle.out_of_route:
            reward = -self.config["out_of_road_penalty"]
        elif vehicle.arrive_destination:
            reward = +self.config["success_reward"]
        return reward, step_info

    def _get_reset_return(self):
        ret = {}
        self.for_each_vehicle(lambda v: v.update_state())
        for v_id, v in self.vehicles.items():
            self.observations[v_id].reset(self, v)
            ret[v_id] = self.observations[v_id].observe(v)
        return ret[DEFAULT_AGENT] if self.num_agents == 1 else ret

    def get_single_observation(self, vehicle_config: "PGConfig") -> "ObservationType":
        if self.config["use_image"]:
            o = ImageStateObservation(vehicle_config)
        else:
            o = LidarStateObservationV2(vehicle_config)
        return o


if __name__ == '__main__':

    def _act(env, action):
        assert env.action_space.contains(action)
        obs, reward, done, info = env.step(action)
        assert env.observation_space.contains(obs)
        assert np.isscalar(reward)
        assert isinstance(info, dict)


    env = PGDriveEnvV2()
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
