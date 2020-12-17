import copy
import json
import logging
import os.path as osp
from typing import Union, Optional

import gym
import numpy as np
from pgdrive.envs.observation_type import LidarStateObservation, ImageStateObservation
from pgdrive.pg_config import PgConfig
from pgdrive.scene_creator.algorithm.BIG import BigGenerateMethod
from pgdrive.scene_creator.ego_vehicle.base_vehicle import BaseVehicle
from pgdrive.scene_creator.ego_vehicle.vehicle_module.depth_camera import DepthCamera
from pgdrive.scene_creator.ego_vehicle.vehicle_module.mini_map import MiniMap
from pgdrive.scene_creator.ego_vehicle.vehicle_module.rgb_camera import RgbCamera
from pgdrive.scene_creator.map import Map, MapGenerateMethod
from pgdrive.scene_manager.traffic_manager import TrafficManager, TrafficMode
from pgdrive.utils import recursive_equal
from pgdrive.world.chase_camera import ChaseCamera
from pgdrive.world.manual_controller import KeyboardController, JoystickController
from pgdrive.world.pg_world import PgWorld

pregenerated_map_file = osp.join(osp.dirname(osp.dirname(osp.abspath(__file__))), "assets", "maps", "PGDrive-maps.json")


class PGDriveEnv(gym.Env):
    @staticmethod
    def default_config() -> PgConfig:
        env_config = dict(

            # ===== Rendering =====
            use_render=False,  # pop a window to render or not
            # force_fps=None,
            debug=False,
            manual_control=False,
            controller="keyboard",  # "joystick" or "keyboard"
            use_chase_camera=True,
            camera_height=1.8,

            # ===== Traffic =====
            traffic_density=0.1,
            traffic_mode=TrafficMode.Add_once,

            # ===== Observation =====
            use_image=False,
            rgb_clip=True,
            vehicle_config=dict(),  # use default vehicle modules see more in BaseVehicle
            image_source="rgb_cam",  # mini_map or rgb_cam or depth cam

            # ===== Map Config =====
            map_config={
                Map.GENERATE_METHOD: BigGenerateMethod.BLOCK_NUM,
                Map.GENERATE_PARA: 3
            },
            load_map_from_json=True,  # Whether to load maps from pre-generated file
            _load_map_from_json=pregenerated_map_file,  # The path to the pre-generated file

            # ===== Generalization =====
            start_seed=0,
            environment_num=1,

            # ===== Action =====
            decision_repeat=5,

            # ===== Reward Scheme =====
            success_reward=20,
            out_of_road_penalty=5,
            crash_penalty=10,
            acceleration_penalty=0.0,
            steering_penalty=0.1,
            low_speed_penalty=0.0,
            driving_reward=1.0,
            general_penalty=0.0,
            speed_reward=0.1,

            # ===== Others =====
            pg_world_config=dict(),
            use_increment_steering=False,
            action_check=False,
        )
        return PgConfig(env_config)

    def __init__(self, config: dict = None):
        self.config = self.default_config()
        if config:
            self.config.update(config)

        # set their value after vehicle created
        vehicle_config = BaseVehicle.get_vehicle_config(self.config["vehicle_config"])
        self.observation = LidarStateObservation(vehicle_config) if not self.config["use_image"] \
            else ImageStateObservation(vehicle_config, self.config["image_source"], self.config["rgb_clip"])
        self.observation_space = self.observation.observation_space
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(2, ), dtype=np.float32)

        self.start_seed = self.config["start_seed"]
        self.env_num = self.config["environment_num"]
        self.map_config = self.config["map_config"]
        self.use_render = self.config["use_render"]
        pg_world_config = self.config["pg_world_config"]
        pg_world_config.update(
            {
                "use_render": self.use_render,
                "use_image": self.config["use_image"],
                "debug": self.config["debug"],
                # "force_fps": self.config["force_fps"],
                "decision_repeat": self.config["decision_repeat"],
            }
        )
        self.pg_world_config = pg_world_config

        # lazy initialization, create the main vehicle in the lazy_init() func
        self.pg_world = None
        self.traffic_manager = None
        self.control_camera = None
        self.controller = None
        self.restored_maps = dict()

        self.maps = {_seed: None for _seed in range(self.start_seed, self.start_seed + self.env_num)}
        self.current_seed = self.start_seed
        self.current_map = None
        self.vehicle = None  # Ego vehicle
        self.done = False

    def lazy_init(self):
        # It is the true init() func to create the main vehicle and its module
        if self.pg_world is not None:
            return

        # init world
        self.pg_world = PgWorld(self.pg_world_config)
        self.pg_world.accept("r", self.reset)
        self.pg_world.accept("escape", self.force_close)

        # init traffic manager
        self.traffic_manager = TrafficManager(self.config["traffic_mode"])

        if self.config["manual_control"]:
            if self.config["controller"] == "keyboard":
                self.controller = KeyboardController()
            elif self.config["controller"] == "joystick":
                self.controller = JoystickController(self.pg_world)
            else:
                raise ValueError("No such a controller type: {}".format(self.config["controller"]))

        # init vehicle
        v_config = self.config["vehicle_config"]
        self.vehicle = BaseVehicle(self.pg_world, v_config)

        # for manual_control and main camera type
        if (self.config["use_render"] or self.config["use_image"]) and self.config["use_chase_camera"]:
            self.control_camera = ChaseCamera(
                self.pg_world.cam, self.vehicle, self.config["camera_height"], 7, self.pg_world
            )
        # add sensors
        self.add_modules_for_vehicle()

        if self.use_render or self.config["use_image"]:
            self.control_camera.reset(self.vehicle.position)

    def step(self, action: np.ndarray):

        if self.config["action_check"]:
            assert self.action_space.contains(action), "Input {} is not compatible with action space {}!".format(
                action, self.action_space
            )

        # prepare step
        if self.config["manual_control"] and self.use_render:
            action = self.controller.process_input()
        self.vehicle.prepare_step(action)
        self.traffic_manager.prepare_step()

        # ego vehicle/ traffic step
        for _ in range(self.config["decision_repeat"]):
            # traffic vehicles step
            self.traffic_manager.step(self.pg_world.pg_config["physics_world_step_size"])
            self.pg_world.step()

        # update states
        self.vehicle.update_state()
        self.traffic_manager.update_state(self.pg_world.physics_world)

        #  panda3d render and garbage collecting loop
        self.pg_world.taskMgr.step()

        obs = self.observation.observe(self.vehicle)
        reward = self.reward(action)
        done_reward, done_info = self._done_episode()
        info = {
            "cost": float(0),
            "velocity": float(self.vehicle.speed),
            "steering": float(self.vehicle.steering),
            "acceleration": float(self.vehicle.throttle_brake),
            "step_reward": float(reward)
        }
        info.update(done_info)
        return obs, reward + done_reward, self.done, info

    def render(self, mode='human', text: Optional[Union[dict, str]] = None) -> Optional[np.ndarray]:
        assert self.use_render or self.config["use_image"], "render is off now, can not render"
        self.pg_world.render_frame(text)
        if mode != "human" and self.config["use_image"]:
            # fetch img from img stack to be make this func compatible with other render func in RL setting
            return self.observation.img_obs.get_image()

        if mode != "human" and self.config["use_render"]:
            if not hasattr(self, "_temporary_img_obs"):
                from pgdrive.envs.observation_type import ImageObservation
                image_source = "rgb_cam"
                self.temporary_img_obs = ImageObservation(self.vehicle.vehicle_config, image_source, False)
            self.temporary_img_obs.observe(self.vehicle.image_sensors[image_source])
            return self.temporary_img_obs.get_image()

        logging.warning("You do not set 'use_image' or 'use_image' to True, so no image will be returned!")
        return None

    def reset(self):
        self.lazy_init()  # it only works the first time when reset() is called to avoid the error when render
        self.done = False

        # clear world and traffic manager
        self.pg_world.clear_world()
        # select_map
        self.select_map()

        # reset main vehicle
        self.vehicle.reset(self.current_map, self.vehicle.born_place, 0.0)

        # generate new traffic according to the map
        self.traffic_manager.generate_traffic(
            self.pg_world, self.current_map, self.vehicle, self.config["traffic_density"]
        )
        o, *_ = self.step(np.array([0.0, 0.0]))
        return o

    def reward(self, action):
        # Reward for moving forward in current lane
        current_lane = self.vehicle.lane
        long_last, _ = current_lane.local_coordinates(self.vehicle.last_position)
        long_now, lateral_now = current_lane.local_coordinates(self.vehicle.position)

        reward = 0.0
        lateral_factor = 1 - 2 * abs(lateral_now) / self.current_map.lane_width
        reward += self.config["driving_reward"] * (long_now - long_last) * lateral_factor

        # Penalty for frequent steering
        steering_change = abs(self.vehicle.last_current_action[0][0] - self.vehicle.last_current_action[1][0])
        steering_penalty = self.config["steering_penalty"] * steering_change * self.vehicle.speed / 20
        reward -= steering_penalty
        # Penalty for frequent acceleration / brake
        acceleration_penalty = self.config["acceleration_penalty"] * ((action[1])**2)
        reward -= acceleration_penalty

        # Penalty for waiting
        low_speed_penalty = 0
        if self.vehicle.speed < 1:
            low_speed_penalty = self.config["low_speed_penalty"]  # encourage car
        reward -= low_speed_penalty

        reward -= self.config["general_penalty"]

        reward += self.config["speed_reward"] * (self.vehicle.speed / self.vehicle.max_speed)

        return reward

    def _done_episode(self) -> (float, dict):
        reward_ = 0
        done_info = dict(crash=False, out_of_road=False, arrive_dest=False)
        long, lat = self.vehicle.routing_localization.final_lane.local_coordinates(self.vehicle.position)

        if self.vehicle.routing_localization.final_lane.length - 5 < long < self.vehicle.routing_localization.final_lane.length + 5 \
                and self.current_map.lane_width / 2 >= lat >= (
                0.5 - self.current_map.lane_num) * self.current_map.lane_width:
            self.done = True
            reward_ += self.config["success_reward"]
            print("arrive_dest")
            done_info["arrive_dest"] = True
        elif self.vehicle.crash:
            self.done = True
            reward_ -= self.config["crash_penalty"]
            print("crash")
            done_info["crash"] = True
        elif self.vehicle.out_of_road or self.vehicle.out_of_road:
            self.done = True
            reward_ -= self.config["out_of_road_penalty"]
            print("out_of_road")
            done_info["out_of_road"] = True

        return reward_, done_info

    def close(self):
        if self.pg_world is not None:
            self.vehicle.destroy(self.pg_world.physics_world)
            self.traffic_manager.destroy(self.pg_world.physics_world)

            del self.traffic_manager
            self.traffic_manager = None

            del self.control_camera
            self.control_camera = None

            del self.controller
            self.controller = None

            del self.vehicle
            self.vehicle = None

            self.pg_world.clear_world()
            self.pg_world.close_world()
            del self.pg_world
            self.pg_world = None

        del self.maps
        self.maps = {_seed: None for _seed in range(self.start_seed, self.start_seed + self.env_num)}
        del self.current_map
        self.current_map = None
        del self.restored_maps
        self.restored_maps = dict()

    def select_map(self):
        if self.config["load_map_from_json"] and self.current_map is None:
            assert self.config["_load_map_from_json"]
            self.load_all_maps_from_json(self.config["_load_map_from_json"])

        # remove map from world before adding
        if self.current_map is not None:
            self.current_map.unload_from_pg_world(self.pg_world)

        # create map
        self.current_seed = np.random.randint(self.start_seed, self.start_seed + self.env_num)
        if self.maps.get(self.current_seed, None) is None:

            if self.config["load_map_from_json"]:
                map_config = self.restored_maps.get(self.current_seed, None)
                assert map_config is not None
            else:
                map_config = self.config["map_config"]
                map_config.update({"seed": self.current_seed})

            new_map = Map(self.pg_world, map_config)
            self.maps[self.current_seed] = new_map
            self.current_map = self.maps[self.current_seed]
        else:
            self.current_map = self.maps[self.current_seed]
            assert isinstance(self.current_map, Map), "map should be an instance of Map() class"
            self.current_map.load_to_pg_world(self.pg_world)

    def add_modules_for_vehicle(self):
        # add vehicle module for training according to config
        vehicle_config = self.vehicle.vehicle_config
        self.vehicle.add_routing_localization(vehicle_config["show_navi_point"])  # default added
        if not self.config["use_image"]:
            # TODO visualize lidar
            self.vehicle.add_lidar(vehicle_config["lidar"][0], vehicle_config["lidar"][1])

            if self.config["use_render"]:
                rgb_cam_config = vehicle_config["rgb_cam"]
                rgb_cam = RgbCamera(rgb_cam_config[0], rgb_cam_config[1], self.vehicle.chassis_np, self.pg_world)
                self.vehicle.add_image_sensor("rgb_cam", rgb_cam)

                mini_map = MiniMap(vehicle_config["mini_map"], self.vehicle.chassis_np, self.pg_world)
                self.vehicle.add_image_sensor("mini_map", mini_map)
            return

        if self.config["use_image"]:
            # 3 types image observation
            if self.config["image_source"] == "rgb_cam":
                rgb_cam_config = vehicle_config["rgb_cam"]
                rgb_cam = RgbCamera(rgb_cam_config[0], rgb_cam_config[1], self.vehicle.chassis_np, self.pg_world)
                self.vehicle.add_image_sensor("rgb_cam", rgb_cam)
            elif self.config["image_source"] == "mini_map":
                mini_map = MiniMap(vehicle_config["mini_map"], self.vehicle.chassis_np, self.pg_world)
                self.vehicle.add_image_sensor("mini_map", mini_map)
            elif self.config["image_source"] == "depth_cam":
                cam_config = vehicle_config["depth_cam"]
                depth_cam = DepthCamera(*cam_config, self.vehicle.chassis_np, self.pg_world)
                self.vehicle.add_image_sensor("depth_cam", depth_cam)
            else:
                raise ValueError("No module named {}".format(self.config["image_source"]))

        # load more sensors for visualization when render, only for beauty...
        if self.config["use_render"]:
            if self.config["image_source"] == "mini_map":
                rgb_cam_config = vehicle_config["rgb_cam"]
                rgb_cam = RgbCamera(rgb_cam_config[0], rgb_cam_config[1], self.vehicle.chassis_np, self.pg_world)
                self.vehicle.add_image_sensor("rgb_cam", rgb_cam)
            else:
                mini_map = MiniMap(vehicle_config["mini_map"], self.vehicle.chassis_np, self.pg_world)
                self.vehicle.add_image_sensor("mini_map", mini_map)

    def dump_all_maps(self):
        assert self.pg_world is None, "We assume you generate map files in independent tasks (not in training). " \
                                      "So you should run the generating script without calling reset of the " \
                                      "environment."

        self.lazy_init()  # it only works the first time when reset() is called to avoid the error when render
        self.pg_world.clear_world()

        for seed in range(self.start_seed, self.start_seed + self.env_num):
            map_config = copy.deepcopy(self.config["map_config"])
            map_config.update({"seed": seed})
            new_map = Map(self.pg_world, map_config)
            self.maps[seed] = new_map
            new_map.unload_from_pg_world(self.pg_world)
            print("Finish generating map with seed: ", seed)

        map_data = dict()
        for seed, map in self.maps.items():
            assert map is not None
            map_data[seed] = map.save_map()

        return_data = dict(map_config=copy.deepcopy(self.config["map_config"]), map_data=copy.deepcopy(map_data))
        return return_data

    def load_all_maps(self, data):
        assert isinstance(data, dict)
        assert set(data.keys()) == set(["map_config", "map_data"])
        assert set(self.maps.keys()).issubset(set([int(v) for v in data["map_data"].keys()]))

        print(
            "Restoring the maps from pre-generated file! "
            "We have {} maps in the file and restoring {} maps range from {} to {}".format(
                len(data["map_data"]), len(self.maps.keys()), min(self.maps.keys()), max(self.maps.keys())
            )
        )

        maps_collection_config = data["map_config"]
        assert set(self.config["map_config"].keys()) == set(maps_collection_config.keys())
        for k in self.config["map_config"]:
            assert maps_collection_config[k] == self.config["map_config"][k]

        # for seed, map_dict in data["map_data"].items():
        for seed in self.maps.keys():
            assert str(seed) in data["map_data"]
            assert self.maps[seed] is None
            map_config = {}
            map_config[Map.GENERATE_METHOD] = MapGenerateMethod.PG_MAP_FILE
            map_config[Map.GENERATE_PARA] = data["map_data"][str(seed)]
            self.restored_maps[seed] = map_config

    def load_all_maps_from_json(self, path):
        assert path.endswith(".json")
        assert osp.isfile(path)
        with open(path, "r") as f:
            restored_data = json.load(f)
        if recursive_equal(self.config["map_config"], restored_data["map_config"]) and \
                self.start_seed + self.env_num < len(restored_data["map_data"]):
            self.load_all_maps(restored_data)
            return True
        else:
            print(
                "Warning: The pre-generated maps is with config {}, but current environment's map "
                "config is {}.\nWe now fallback to BIG algorithm to generate map online!".format(
                    restored_data["map_config"], self.map_config
                )
            )
            self.config["load_map_from_json"] = False  # Don't fall into this function again.
            return False

    def force_close(self):
        self.close()
        raise KeyboardInterrupt()

    def set_current_seed(self, seed):
        self.current_seed = seed

    def get_map(self):
        return self.current_map.get_map_image_array()

    def get_vehicle_num(self):
        if self.traffic_manager is None:
            return 0
        return self.traffic_manager.get_vehicle_num()
