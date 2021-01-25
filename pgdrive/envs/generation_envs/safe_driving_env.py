from pgdrive.envs import PGDriveEnv
from pgdrive.scene_creator.highway_vehicle.behavior import IDMVehicle
from pgdrive.pg_config import PGConfig


class SafeDrivingEnv(PGDriveEnv):

    def default_config(self) -> PGConfig:
        """
        Now we construct a simple scene to verify ideas quickly
        TODO
        Train/Test set both contain 10 maps
        :return: PGConfig
        """
        config = super(SafeDrivingEnv, self).default_config()
        config.extend_config_with_unknown_keys(dict(environment_num=1,
                                                    start_seed=1,
                                                    map="rCCXC",

                                                    # traffic setting
                                                    random_traffic=False,
                                                    traffic_density=0.15,
                                                    traffic_mode="trigger",

                                                    # reward setting under saver mode
                                                    use_general_takeover_penalty=False,
                                                    takeover_penalty=0.5,
                                                    speed_reward=0.5,

                                                    # saver config
                                                    use_saver=True,
                                                    out_of_road_constrain=True,
                                                    crash_constrain=True,
                                                    save_level=0.4))
        return config

    def reward(self, action):
        current_lane = self.vehicle.lane
        long_last, _ = current_lane.local_coordinates(self.vehicle.last_position)
        long_now, lateral_now = current_lane.local_coordinates(self.vehicle.position)

        reward = 0.0
        if abs(lateral_now) <= self.current_map.lane_width / 2:
            # Out of road will get no reward
            reward += self.config["driving_reward"] * (long_now - long_last)
            reward += self.config["speed_reward"] * (self.vehicle.speed / self.vehicle.max_speed)

        # Penalty for waiting
        if self.vehicle.speed < 1:
            reward -= self.config["low_speed_penalty"]  # encourage car
        reward -= self.config["general_penalty"]

        self.step_info["raw_step_reward"] = reward

        if self.save_mode and self.config["use_general_takeover_penalty"]:
            # takeover means the situation is dangerous, so give a penalty in every takeover step
            reward -= self.config["takeover_penalty"]
        return reward

    def _done_episode(self) -> (float, dict):
        done_reward = super(SafeDrivingEnv, self)._done_episode()
        if not self.config["use_saver"]:
            return done_reward

        if self.step_info["out_of_road"] and not self.config["out_of_road_constrain"] and \
                not self.step_info["arrive_dest"]:
            # episode will not be done when out of road, since expert can save it
            self.done = False
        if self.step_info["crash"] and not self.config["crash_constrain"] and not self.step_info["arrive_dest"]:
            self.done = False
        return done_reward

    def custom_info_callback(self):
        self.step_info["high_speed"] = True if self.vehicle.speed >= IDMVehicle.MAX_SPEED else False
