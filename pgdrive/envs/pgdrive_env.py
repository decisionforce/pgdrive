import copy
import logging
import os.path as osp
from typing import Union, Dict, AnyStr, Tuple

import numpy as np

from pgdrive.component.blocks.first_block import FirstPGBlock
from pgdrive.component.map.base_map import BaseMap, MapGenerateMethod, parse_map_config
from pgdrive.component.map.pg_map import PGMap
from pgdrive.component.vehicle.base_vehicle import BaseVehicle
from pgdrive.constants import DEFAULT_AGENT, TerminationState
from pgdrive.engine.core.manual_controller import KeyboardController, JoystickController
from pgdrive.engine.engine_utils import engine_initialized
from pgdrive.engine.engine_utils import set_global_random_seed
from pgdrive.envs.base_env import BasePGDriveEnv
from pgdrive.manager.traffic_manager import TrafficMode
from pgdrive.obs.image_obs import ImageStateObservation
from pgdrive.obs.state_obs import LidarStateObservation
from pgdrive.utils import clip, Config, concat_step_infos
from pgdrive.utils import get_np_random

pregenerated_map_file = osp.join(osp.dirname(osp.dirname(osp.abspath(__file__))), "assets", "maps", "PGDrive-maps.json")

PGDriveEnvV1_DEFAULT_CONFIG = dict(
    # ===== Generalization =====
    start_seed=0,
    environment_num=1,

    # ===== Map Config =====
    map=3,  # int or string: an easy way to fill map_config
    map_config={
        BaseMap.GENERATE_TYPE: MapGenerateMethod.BIG_BLOCK_NUM,
        BaseMap.GENERATE_CONFIG: None,  # it can be a file path / block num / block ID sequence
        BaseMap.LANE_WIDTH: 3.5,
        BaseMap.LANE_NUM: 3,
        BaseMap.SEED: 10,
        "draw_map_resolution": 1024,  # Drawing the map in a canvas of (x, x) pixels.
        "block_type_version": "v1",
        "exit_length": 50,
    },
    load_map_from_json=True,  # Whether to load maps from pre-generated file
    _load_map_from_json=pregenerated_map_file,  # The path to the pre-generated file

    # ===== Observation =====
    use_topdown=False,  # Use top-down view
    offscreen_render=False,
    _disable_detector_mask=False,

    # ===== Traffic =====
    traffic_density=0.1,
    traffic_mode=TrafficMode.Trigger,  # "Respawn", "Trigger", "Hybrid"
    random_traffic=False,  # Traffic is randomized at default.

    # ===== Object =====
    accident_prob=0.,  # accident may happen on each block with this probability, except multi-exits block

    # ===== Others =====
    auto_termination=True,  # Whether to done the environment after 250*(num_blocks+1) steps.

    # ===== Single-agent vehicle config =====
    vehicle_config=dict(
        # ===== vehicle module config =====
        # laser num, distance, other vehicle info num
        lidar=dict(num_lasers=240, distance=50, num_others=4, gaussian_noise=0.0, dropout_prob=0.0),
        show_lidar=False,
        mini_map=(84, 84, 250),  # buffer length, width
        rgb_camera=(84, 84),  # buffer length, width
        depth_camera=(84, 84, True),  # buffer length, width, view_ground
        side_detector=dict(num_lasers=0, distance=50),  # laser num, distance
        show_side_detector=False,
        lane_line_detector=dict(num_lasers=0, distance=20),  # laser num, distance
        show_lane_line_detector=False,

        # ===== use image =====
        image_source="rgb_camera",  # take effect when only when offscreen_render == True

        # ===== vehicle spawn and destination =====
        spawn_lane_index=(FirstPGBlock.NODE_1, FirstPGBlock.NODE_2, 0),
        destination_lane_index=None,
        spawn_longitude=5.0,
        spawn_lateral=0.0,
        destination_node=None,

        # ==== others ====
        overtake_stat=False,  # we usually set to True when evaluation
        action_check=False,
        use_saver=False,
        save_level=0.5,
        # vehicle_length=4,
        # vehicle_width=1.5,
        random_color=False,
    ),
    rgb_clip=True,

    # ===== Reward Scheme =====
    success_reward=20,
    out_of_road_penalty=5,
    crash_vehicle_penalty=10,
    crash_object_penalty=2,
    acceleration_penalty=0.0,
    steering_penalty=0.1,
    low_speed_penalty=0.0,
    driving_reward=1.0,
    general_penalty=0.0,
    speed_reward=0.1,

    # ===== Cost Scheme =====
    crash_vehicle_cost=1,
    crash_object_cost=1,
    out_of_road_cost=1.
)


class PGDriveEnv(BasePGDriveEnv):
    @classmethod
    def default_config(cls) -> "Config":
        config = super(PGDriveEnv, cls).default_config()
        config.update(PGDriveEnvV1_DEFAULT_CONFIG)
        config.register_type("map", str, int)
        config["map_config"].register_type("config", None)
        return config

    def __init__(self, config: dict = None):
        super(PGDriveEnv, self).__init__(config)

    def _merge_extra_config(self, config: Union[dict, "Config"]) -> "Config":
        """Check, update, sync and overwrite some config."""
        config = self.default_config().update(config, allow_add_new_key=False)
        if config["vehicle_config"]["lidar"]["distance"] > 50:
            config["max_distance"] = config["vehicle_config"]["lidar"]["distance"]
        return config

    def _post_process_config(self, config):
        if not config["rgb_clip"]:
            logging.warning(
                "You have set rgb_clip = False, which means the observation will be uint8 values in [0, 255]. "
                "Please make sure you have parsed them later before feeding them to network!"
            )
        config["map_config"] = parse_map_config(
            easy_map_config=config["map"], new_map_config=config["map_config"], default_config=self.default_config_copy
        )
        return config

    def _after_lazy_init(self):
        if self.config["manual_control"]:
            if self.config["controller"] == "keyboard":
                self.controller = KeyboardController()
            elif self.config["controller"] == "joystick":
                self.controller = JoystickController()
            else:
                raise ValueError("No such a controller type: {}".format(self.config["controller"]))

        # initialize track vehicles
        vehicles = self.agent_manager.get_vehicle_list()
        current_track_vehicle = vehicles[0]

        # for manual_control and main camera type
        if self.config["use_render"] or self.config["offscreen_render"]:
            self.main_camera.set_follow_lane(self.config["use_chase_camera_follow_lane"])
            self.main_camera.track(current_track_vehicle)
            self.engine.accept("b", self.bird_view_camera)
        self.engine.accept("q", self.chase_camera)

    def _get_observations(self):
        return {self.DEFAULT_AGENT: self.get_single_observation(self.config["vehicle_config"])}

    def _preprocess_actions(self, actions: Union[np.ndarray, Dict[AnyStr, np.ndarray]]) \
            -> Tuple[Union[np.ndarray, Dict[AnyStr, np.ndarray]], Dict]:
        if self.config["manual_control"] and self.config["use_render"] \
                and self.current_track_vehicle in self.agent_manager.get_vehicle_list() and not self.main_camera.is_bird_view_camera():
            action = self.controller.process_input(self.current_track_vehicle)
            if self.is_multi_agent:
                actions[self.agent_manager.object_to_agent(self.current_track_vehicle.name)] = action
            else:
                actions = action

        if not self.is_multi_agent:
            actions = {v_id: actions for v_id in self.vehicles.keys()}
        else:
            if self.config["vehicle_config"]["action_check"]:
                # Check whether some actions are not provided.
                given_keys = set(actions.keys())
                have_keys = set(self.vehicles.keys())
                assert given_keys == have_keys, "The input actions: {} have incompatible keys with existing {}!".format(
                    given_keys, have_keys
                )
            else:
                # That would be OK if extra actions is given. This is because, when evaluate a policy with naive
                # implementation, the "termination observation" will still be given in T=t-1. And at T=t, when you
                # collect action from policy(last_obs) without masking, then the action for "termination observation"
                # will still be computed. We just filter it out here.
                actions = {v_id: actions[v_id] for v_id in self.vehicles.keys()}

        saver_info = dict()
        for v_id, v in self.vehicles.items():
            actions[v_id], saver_info[v_id] = self.saver(v_id, actions)
        return actions, saver_info

    def _get_step_return(self, actions, step_infos):
        """Don't need to copy anything here!"""
        # update obs, dones, rewards, costs, calculate done at first !
        obses = {}
        done_infos = {}
        cost_infos = {}
        reward_infos = {}
        rewards = {}
        for v_id, v in self.vehicles.items():
            obses[v_id] = self.observations[v_id].observe(v)
            done_function_result, done_infos[v_id] = self.done_function(v_id)
            rewards[v_id], reward_infos[v_id] = self.reward_function(v_id)
            _, cost_infos[v_id] = self.cost_function(v_id)
            done = done_function_result or self.dones[v_id]
            self.dones[v_id] = done

        should_done = self.config["auto_termination"] and (self.episode_steps >= (self.current_map.num_blocks * 250))

        termination_infos = self.for_each_vehicle(_auto_termination, should_done)

        step_infos = concat_step_infos([
            step_infos,
            done_infos,
            reward_infos,
            cost_infos,
            termination_infos,
        ])

        if should_done:
            for k in self.dones:
                self.dones[k] = True

        dones = {k: self.dones[k] for k in self.vehicles.keys()}
        for v_id, r in rewards.items():
            self.episode_rewards[v_id] += r
            step_infos[v_id]["episode_reward"] = self.episode_rewards[v_id]
            self.episode_lengths[v_id] += 1
            step_infos[v_id]["episode_length"] = self.episode_lengths[v_id]
        if not self.is_multi_agent:
            return self._wrap_as_single_agent(obses), self._wrap_as_single_agent(rewards), \
                   self._wrap_as_single_agent(dones), self._wrap_as_single_agent(step_infos)
        else:
            return obses, rewards, dones, step_infos

    def done_function(self, vehicle_id: str):
        vehicle = self.vehicles[vehicle_id]
        done = False
        done_info = dict(crash=False, crash_vehicle=False, crash_object=False, out_of_road=False, arrive_dest=False)
        if vehicle.arrive_destination:
            done = True
            logging.info("Episode ended! Reason: arrive_dest.")
            done_info[TerminationState.SUCCESS] = True
        elif vehicle.crash_vehicle:
            done = True
            logging.info("Episode ended! Reason: crash. ")
            done_info[TerminationState.CRASH_VEHICLE] = True
        elif vehicle.out_of_route or not vehicle.on_lane or vehicle.crash_sidewalk:
            done = True
            logging.info("Episode ended! Reason: out_of_road.")
            done_info[TerminationState.OUT_OF_ROAD] = True
        elif vehicle.crash_object:
            done = True
            done_info[TerminationState.CRASH_OBJECT] = True

        # for compatibility
        # crash almost equals to crashing with vehicles
        done_info[TerminationState.CRASH
        ] = done_info[TerminationState.CRASH_VEHICLE] or done_info[TerminationState.CRASH_OBJECT]
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

    def reward_function(self, vehicle_id):
        """
        Override this func to get a new reward function
        :param vehicle_id: id of BaseVehicle
        :return: reward
        """
        vehicle = self.vehicles[vehicle_id]
        step_info = dict()
        action = vehicle.last_current_action[1]
        # Reward for moving forward in current lane
        current_lane = vehicle.lane if vehicle.lane in vehicle.navigation.current_ref_lanes else \
            vehicle.navigation.current_ref_lanes[0]
        long_last, _ = current_lane.local_coordinates(vehicle.last_position)
        long_now, lateral_now = current_lane.local_coordinates(vehicle.position)

        # reward for lane keeping, without it vehicle can learn to overtake but fail to keep in lane
        reward = 0.0
        lateral_factor = clip(1 - 2 * abs(lateral_now) / vehicle.navigation.get_current_lane_width(), 0.0, 1.0)
        reward += self.config["driving_reward"] * (long_now - long_last) * lateral_factor

        # Penalty for frequent steering
        steering_change = abs(vehicle.last_current_action[0][0] - vehicle.last_current_action[1][0])
        steering_penalty = self.config["steering_penalty"] * steering_change * vehicle.speed / 20
        reward -= steering_penalty

        # Penalty for frequent acceleration / brake
        acceleration_penalty = self.config["acceleration_penalty"] * ((action[1]) ** 2)
        reward -= acceleration_penalty

        # Penalty for waiting
        low_speed_penalty = 0
        if vehicle.speed < 1:
            low_speed_penalty = self.config["low_speed_penalty"]  # encourage car
        reward -= low_speed_penalty
        reward -= self.config["general_penalty"]

        reward += self.config["speed_reward"] * (vehicle.speed / vehicle.max_speed)
        step_info["step_reward"] = reward

        # for done
        if vehicle.arrive_destination:
            reward += self.config["success_reward"]
        elif vehicle.out_of_route:
            reward -= self.config["out_of_road_penalty"]
        elif vehicle.crash_vehicle:
            reward -= self.config["crash_vehicle_penalty"]
        elif vehicle.crash_object:
            reward -= self.config["crash_object_penalty"]

        return reward, step_info

    def _get_reset_return(self):
        ret = {}
        self.engine.after_step()
        for v_id, v in self.vehicles.items():
            self.observations[v_id].reset(self, v)
            ret[v_id] = self.observations[v_id].observe(v)
        return ret if self.is_multi_agent else ret[DEFAULT_AGENT]

    def _update_map(self, episode_data: dict = None):
        map_manager = self.engine.map_manager
        if episode_data is not None:
            # TODO restore/replay here
            # Since in episode data map data only contains one map, values()[0] is the map_parameters
            map_data = episode_data["map_data"].values()
            assert len(map_data) > 0, "Can not find map info in episode data"
            blocks_info = map_data[0]

            map_config = self.config["map_config"].copy()
            map_config[BaseMap.GENERATE_TYPE] = MapGenerateMethod.PG_MAP_FILE
            map_config[BaseMap.GENERATE_CONFIG] = blocks_info
            map_manager.spawn_object(PGMap, map_config=map_config)
            return

        if self.config["load_map_from_json"] and self.current_map is None:
            assert self.config["_load_map_from_json"]
            map_manager.read_all_maps_from_json(self.config["_load_map_from_json"])

        # remove map from world before adding
        if self.current_map is not None:
            map_manager.unload_map(self.current_map)

        if map_manager.pg_maps[self.current_seed] is None:
            if self.config["load_map_from_json"]:
                map_config = map_manager.restored_pg_map_configs.get(self.current_seed, None)
                assert map_config is not None
            else:
                map_config = self.config["map_config"]
                map_config.update({"seed": self.current_seed})
            map = map_manager.spawn_object(PGMap, map_config=map_config)
        else:
            map = map_manager.pg_maps[self.current_seed]
        map_manager.load_map(map)

    def dump_all_maps(self):
        assert not engine_initialized(), \
            "We assume you generate map files in independent tasks (not in training). " \
            "So you should run the generating script without calling reset of the " \
            "environment."

        self.lazy_init()  # it only works the first time when reset() is called to avoid the error when render
        assert engine_initialized()

        for seed in range(self.start_seed, self.start_seed + self.env_num):
            map_config = copy.deepcopy(self.config["map_config"])
            map_config.update({"seed": seed})
            set_global_random_seed(seed)
            new_map = self.engine.map_manager.spawn_object(PGMap, map_config=map_config)
            self.engine.map_manager.unload_map(new_map)
            logging.info("Finish generating map with seed: {}".format(seed))

        map_data = dict()
        for seed, map in self.maps.items():
            assert map is not None
            map_data[seed] = map.save_map()

        return_data = dict(map_config=self.config["map_config"].copy().get_dict(), map_data=copy.deepcopy(map_data))
        return return_data

    def toggle_expert_takeover(self):
        """
        Only take effect whene vehicle num==1
        :return: None
        """
        self.current_track_vehicle._expert_takeover = not self.current_track_vehicle._expert_takeover

    def chase_camera(self) -> (str, BaseVehicle):
        if self.main_camera is None:
            return
        self.main_camera.reset()
        if self.config["prefer_track_agent"] is not None and self.config["prefer_track_agent"] in self.vehicles.keys():
            new_v = self.vehicles[self.config["prefer_track_agent"]]
            current_track_vehicle = new_v
        else:
            if self.main_camera.is_bird_view_camera():
                current_track_vehicle = self.current_track_vehicle
            else:
                vehicles = list(self.agent_manager.active_agents.values())
                if len(vehicles) <= 1:
                    return
                if self.current_track_vehicle in vehicles:
                    vehicles.remove(self.current_track_vehicle)
                new_v = get_np_random().choice(vehicles)
                current_track_vehicle = new_v
        self.main_camera.track(current_track_vehicle)
        return

    def bird_view_camera(self):
        self.main_camera.stop_track()

    def saver(self, v_id: str, actions):
        """
        Rule to enable saver
        :param v_id: id of a vehicle
        :param vehicle:BaseVehicle that need protection of saver
        :param actions: original actions of all vehicles
        :return: a new action to override original action
        """
        vehicle = self.vehicles[v_id]
        action = actions[v_id]
        steering = action[0]
        throttle = action[1]
        if vehicle.config["use_saver"] or vehicle._expert_takeover:
            # saver can be used for human or another AI
            save_level = vehicle.config["save_level"] if not vehicle._expert_takeover else 1.0
            obs = self.observations[v_id].observe(vehicle)
            from pgdrive.examples.ppo_expert import expert
            try:
                saver_a = expert(obs, deterministic=False)
            except ValueError:
                print("Expert can not takeover, due to observation space mismathing!")
                saver_a = action
            else:
                if save_level > 0.9:
                    steering = saver_a[0]
                    throttle = saver_a[1]
                elif save_level > 1e-3:
                    heading_diff = vehicle.heading_diff(vehicle.lane) - 0.5
                    f = min(1 + abs(heading_diff) * vehicle.speed * vehicle.max_speed, save_level * 10)
                    # for out of road
                    if (obs[0] < 0.04 * f and heading_diff < 0) or (obs[1] < 0.04 * f and heading_diff > 0) or obs[
                        0] <= 1e-3 or \
                            obs[
                                1] <= 1e-3:
                        steering = saver_a[0]
                        throttle = saver_a[1]
                        if vehicle.speed < 5:
                            throttle = 0.5
                    # if saver_a[1] * vehicle.speed < -40 and action[1] > 0:
                    #     throttle = saver_a[1]

                    # for collision
                    lidar_p = env.observations[DEFAULT_AGENT].cloud_points
                    left = int(vehicle.lidar.num_lasers / 4)
                    right = int(vehicle.lidar.num_lasers / 4 * 3)
                    if min(lidar_p[left - 4:left + 6]) < (save_level + 0.1) / 10 or min(lidar_p[right - 4:right + 6]
                                                                                        ) < (save_level + 0.1) / 10:
                        # lateral safe distance 2.0m
                        steering = saver_a[0]
                    if action[1] >= 0 and saver_a[1] <= 0 and min(min(lidar_p[0:10]), min(lidar_p[-10:])) < save_level:
                        # longitude safe distance 15 m
                        throttle = saver_a[1]

        # indicate if current frame is takeover step
        pre_save = vehicle.takeover
        vehicle.takeover = True if action[0] != steering or action[1] != throttle else False
        saver_info = {
            "takeover_start": True if not pre_save and vehicle.takeover else False,
            "takeover_end": True if pre_save and not vehicle.takeover else False,
            "takeover": vehicle.takeover if pre_save else False
        }
        return (steering, throttle) if saver_info["takeover"] else action, saver_info

    def get_single_observation(self, vehicle_config: "Config") -> "ObservationType":
        if self.config["offscreen_render"]:
            o = ImageStateObservation(vehicle_config)
        else:
            o = LidarStateObservation(vehicle_config)
        return o

    def setup_engine(self):
        super(PGDriveEnv, self).setup_engine()
        # Press t can let expert take over. But this function is still experimental.
        self.engine.accept("t", self.toggle_expert_takeover)

        from pgdrive.manager.object_manager import TrafficSignManager
        from pgdrive.manager.traffic_manager import TrafficManager
        self.engine.register_manager("traffic_manager", TrafficManager())
        self.engine.register_manager("object_manager", TrafficSignManager())

    @property
    def main_camera(self):
        return self.engine.main_camera

    @property
    def current_track_vehicle(self):
        return self.engine.current_track_vehicle


def _auto_termination(vehicle, should_done):
    return {TerminationState.MAX_STEP: True if should_done else False}


if __name__ == '__main__':

    def _act(env, action):
        assert env.action_space.contains(action)
        obs, reward, done, info = env.step(action)
        assert env.observation_space.contains(obs)
        assert np.isscalar(reward)
        assert isinstance(info, dict)


    env = PGDriveEnv()
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
