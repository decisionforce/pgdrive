from pg_drive import GeneralizationRacing
from gym import Wrapper
import gym
import numpy as np
from pg_drive.scene_creator.ego_vehicle.vehicle_module.mini_map import MiniMap
from pg_drive.scene_creator.ego_vehicle.vehicle_module.rgb_camera import RgbCamera
from pg_drive.scene_creator.ego_vehicle.vehicle_module.depth_camera import DepthCamera
from pg_drive.envs.observation_type import LidarStateObservation, ImageStateObservation
from pg_drive.pg_config.pg_config import PgConfig
from pg_drive.scene_creator.algorithm.BIG import BigGenerateMethod
from pg_drive.scene_creator.ego_vehicle.base_vehicle import BaseVehicle
from pg_drive.scene_creator.map import Map, MapGenerateMethod
from pg_drive.scene_manager.traffic_manager import TrafficManager, TrafficMode
from pg_drive.world.pg_world import PgWorld
from pg_drive.world.chase_camera import ChaseCamera
from pg_drive.world.manual_controller import KeyboardController, JoystickController
import copy
import json
import os.path as osp
from pg_drive.utils import recursive_equal
from gym.spaces import Box


class ActionRepeatWrapper(GeneralizationRacing):

    @staticmethod
    def default_config() -> PgConfig:
        config = GeneralizationRacing.default_config()
        config.add("max_action_repeat", 100)
        config.add("min_action_repeat", 1)
        return config

    def __init__(self, config: dict = None):
        super(ActionRepeatWrapper, self).__init__(config)
        self.action_space = Box(shape=(self.action_space.shape[0] + 1,), high=self.action_space.high[0],
                                low=self.action_space.low[0], dtype=self.action_space.dtype)
        self.low = self.action_space.low[0]
        self.high = self.action_space.high[0]
        self.action_repeat_low = self.config["min_action_repeat"]
        self.action_repeat_high = self.config["max_action_repeat"]

    def step(self, action):
        action_repeat = action[-1]
        action_repeat = round(
                (action_repeat - self.low) / (self.high - self.low) * (self.action_repeat_high - self.action_repeat_low)
                + self.action_repeat_low
        )

        for repeat in range(action_repeat):
            o, r,



if __name__ == '__main__':
    env = ActionRepeatWrapper()
    env.reset()
    env.step(env.action_space.sample())
    env.close()
