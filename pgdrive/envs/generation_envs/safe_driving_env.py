from pgdrive.envs import PGDriveEnv
from pgdrive.pg_config import PGConfig


class SafeDrivingEnv(PGDriveEnv):

    def default_config(self) -> PGConfig:
        """
        6 block with 0.1 density hybrid traffic
        Currently, Train/Test set both contain 10 maps
        :return: PGConfig
        """
        config = super(SafeDrivingEnv, self).default_config()
        config.extend_config_with_unknown_keys(dict(takeover_penalty=10,
                                                    start_seed=1000,
                                                    environment_num=10,
                                                    traffic_density=0.2,
                                                    traffic_mode="hybrid",
                                                    map=6,
                                                    save_level=0.5,
                                                    use_saver=True))
        return config

    def reward(self, action):
        current_lane = self.vehicle.lane
        long_last, _ = current_lane.local_coordinates(self.vehicle.last_position)
        long_now, lateral_now = current_lane.local_coordinates(self.vehicle.position)

        reward = 0.0
        reward += self.config["driving_reward"] * (long_now - long_last)

        # Penalty for waiting
        if self.vehicle.speed < 1:
            reward -= self.config["low_speed_penalty"]  # encourage car
        reward -= self.config["general_penalty"]
        reward += self.config["speed_reward"] * (self.vehicle.speed / self.vehicle.max_speed)
        self.step_info["raw_step_reward"] = reward

        # this penalty should be added to last step, so I did this in postprocess

        # if self.step_info["save_current"]:
        #     # takeover means the situation is dangerous, so give a penalty
        #     reward -= self.config["takeover_penalty"]
        return reward

    # def _done_episode(self) -> (float, dict):
    #     reward, info = super(SafeDrivingEnv, self)._done_episode()
    #     if not self.config["use_saver"]:
    #         return reward, info
    #     if info["out_of_road"] and not info["crash"] and not info["arrive_dest"] and not self.vehicle.crash_side_walk:
    #         # episode will not be done when out of road, since expert can save it
    #         self.done = False
    #     return reward, info
