from pgdrive.envs import PGDriveEnv
from pgdrive.scene_creator.highway_vehicle.behavior import IDMVehicle
from pgdrive.pg_config import PGConfig


class SafeDrivingEnv(PGDriveEnv):

    def default_config(self) -> PGConfig:
        """
        Train/Test set both contain 10 maps
        :return: PGConfig
        """
        config = super(SafeDrivingEnv, self).default_config()
        config.extend_config_with_unknown_keys(dict(environment_num=1,
                                                    start_seed=1,
                                                    map="rCCXC",
                                                    safe_rl_env=False,

                                                    # traffic setting
                                                    random_traffic=False,
                                                    traffic_density=0.15,
                                                    traffic_mode="trigger",

                                                    # general reward setting
                                                    speed_reward=0.5,

                                                    # cost setting, it will be written in info by default for safe rl
                                                    takeover_cost=5,
                                                    crash_cost=5,
                                                    out_of_road_cost=1,

                                                    # saver config, save_level:0 = use_saver:False, save_level:1=expert
                                                    use_saver=True,
                                                    save_level=0.4))
        return config

    def _get_reset_return(self):
        # pre set
        if self.config["save_level"] > 0.99:
            # 1.0 full takeover
            self.takeover_start = True
        return super(SafeDrivingEnv, self)._get_reset_return()

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

        return reward

    def _done_episode(self) -> (float, dict):
        done_reward = super(SafeDrivingEnv, self)._done_episode()
        if not self.config["use_saver"]:
            return done_reward

        if self.step_info["out_of_road"] and self.config["safe_rl_env"] and \
                not self.step_info["arrive_dest"]:
            # episode will not be done when out of road, since expert can save it
            self.done = False
            self.step_info["native_cost"] = self.config["out_of_road_cost"]
        if self.step_info["crash"] and self.config["safe_rl_env"] and not self.step_info["arrive_dest"]:
            # only max_step will terminate the whole episode in safe_rl_env
            self.done = False
            self.step_info["native_cost"] = self.config["crash_cost"]
        return done_reward

    def custom_info_callback(self):
        super(SafeDrivingEnv, self).custom_info_callback()
        self.step_info["high_speed"] = True if self.vehicle.speed >= IDMVehicle.MAX_SPEED else False
        self.step_info["takeover_cost"] = self.config["takeover_cost"] if self.step_info["takeover_start"] else 0
