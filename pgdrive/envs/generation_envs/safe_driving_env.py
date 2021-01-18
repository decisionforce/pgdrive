from pgdrive.envs import PGDriveEnv
from pgdrive.pg_config import PgConfig
import logging


class SafeDrivingEnv(PGDriveEnv):

    def default_config(self) -> PgConfig:
        config = super(SafeDrivingEnv, self).default_config()
        config["use_saver"] = True
        config["out_of_road_penalty"] = 3
        config["map"] = "rXCORCTS"
        config["traffic_density"] = 0.2
        config["environment_num"] = 1
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
        return reward

    # def _done_episode(self) -> (float, dict):
    #     reward, info = super(SafeDrivingEnv, self)._done_episode()
    #     if not self.config["use_saver"]:
    #         return reward, info
    #     if info["out_of_road"] and not info["crash"] and not info["arrive_dest"] and not self.vehicle.crash_side_walk:
    #         # episode will not be done when out of road, since expert can save it
    #         self.done = False
    #     return reward, info
