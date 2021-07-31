import math
import time
from collections import deque
from typing import Union, Optional

import gym
import numpy as np
import seaborn as sns
from panda3d.bullet import BulletVehicle, BulletBoxShape, ZUp, BulletGhostNode
from panda3d.core import Material, Vec3, TransformState, NodePath, LQuaternionf, BitMask32, TextNode

from pgdrive.component.base_object import BaseObject
from pgdrive.component.lane.abs_lane import AbstractLane
from pgdrive.component.lane.circular_lane import CircularLane
from pgdrive.component.lane.straight_lane import StraightLane
from pgdrive.component.lane.waypoint_lane import WayPointLane
from pgdrive.component.map.base_map import BaseMap
from pgdrive.component.road.road import Road
from pgdrive.component.vehicle_module import Lidar, MiniMap
from pgdrive.component.vehicle_module.depth_camera import DepthCamera
from pgdrive.component.vehicle_module.distance_detector import SideDetector, LaneLineDetector
from pgdrive.component.vehicle_module.rgb_camera import RGBCamera
from pgdrive.component.vehicle_module.routing_localization import RoutingLocalizationModule
from pgdrive.component.vehicle_module.vehicle_panel import VehiclePanel
from pgdrive.constants import RENDER_MODE_ONSCREEN, COLOR, COLLISION_INFO_COLOR, BodyName, CamMask, CollisionGroup
from pgdrive.engine.asset_loader import AssetLoader
from pgdrive.engine.core.image_buffer import ImageBuffer
from pgdrive.engine.core.physics_world import PhysicsWorld
from pgdrive.engine.physics_node import BaseVehicleNode
from pgdrive.utils import get_np_random, Config, safe_clip_for_small_array, Vector
from pgdrive.utils.coordinates_shift import panda_position, pgdrive_position, panda_heading, pgdrive_heading
from pgdrive.utils.engine_utils import get_engine
from pgdrive.utils.math_utils import get_vertical_vector, norm, clip
from pgdrive.utils.scene_utils import ray_localization
from pgdrive.utils.space import ParameterSpace, Parameter, VehicleParameterSpace

DEFAULT_VEHICLE_CONFIG = dict(
    # Control
    increment_steering=False,
    enable_reverse=False,

    # Kinematics
    wheel_friction=0.6,
    max_engine_force=500,
    max_brake_force=40,
    max_steering=40,
    max_speed=120,

    # Observation
    extra_action_dim=0,

    # Render related
    model_type="ego",
    model_details={},
    am_i_the_special_one=False,  # A flag used to paint specified color to the special vehicles.
    random_navi_mark_color=False,
    show_dest_mark=False,
    show_line_to_dest=False,
    show_navi_mark=True,  # TODO(pzh): Why we put this flag here??

    # Spawn points (only used for traffic vehicle!)
    spawn_longitude=0.0,
    spawn_lateral=0.0,
)

# TODO(pzh): Remove model type to other places! We should have a unify model selection flow.

factor = 1

MODEL_TYPES = dict(
    ego=dict(
        length=4,
        width=1.5,
        height=1,
        model=dict(
            path='models/ferra/scene.gltf',
            scale=None,
            offset=None,
            heading=None,
        )
    ),
    s=dict(
        length=3.2,
        width=1.8,
        height=1.5,
        model=dict(
            path='models/new/beetle/scene.gltf',
            scale=(factor * .008, factor * .006, factor * .0062),
            offset=(-0.7, 0, factor * -0.16),
            heading=-90,
        )
    ),
    m=dict(
        length=3.9,
        width=2.0,
        height=1.3,
        model=dict(
            path='models/new/130/scene.gltf',
            scale=(factor * .0055, factor * .0046, factor * .0049),
            offset=(0, 0, factor * 0.33),
            heading=90,
        )
    ),
    l=dict(
        length=4.8,
        width=1.8,
        height=1.9,
        model=dict(
            path='models/new/lada/scene.gltf',
            scale=(factor * 1.1, factor * 1.1, factor * 1.1),
            offset=(1.1, -13.5, factor * -0.046),
            heading=223,
        )
    ),
    xl=dict(
        length=7.3,
        width=2.3,
        height=2.7,
        model=dict(
            path='models/new/truck/scene.gltf',
            scale=(factor * 0.031, factor * 0.025, factor * 0.025),
            offset=(0.35, 0, factor * 0),
            heading=0,
        )
    ),
)


class BaseVehicle(BaseObject):
    default_config = Config(DEFAULT_VEHICLE_CONFIG)

    MODEL = None
    """
    Vehicle chassis and its wheels index
                    0       1
                    II-----II
                        |
                        |  <---chassis
                        |
                    II-----II
                    2       3
    """
    PARAMETER_SPACE = ParameterSpace(
        VehicleParameterSpace.BASE_VEHICLE
    )  # it will not sample config from parameter space
    COLLISION_MASK = CollisionGroup.EgoVehicle
    STEERING_INCREMENT = 0.05

    LENGTH = None
    WIDTH = None

    # for random color
    MATERIAL_COLOR_COEFF = 10  # to resist other factors, since other setting may make color dark
    MATERIAL_METAL_COEFF = 1  # 0-1
    MATERIAL_ROUGHNESS = 0.8  # smaller to make it more smooth, and reflect more light
    MATERIAL_SHININESS = 1  # 0-128 smaller to make it more smooth, and reflect more light
    MATERIAL_SPECULAR_COLOR = (3, 3, 3, 3)

    def __init__(
            self,
            vehicle_config: Union[dict, Config] = None,
            name: str = None,
            am_i_the_special_one=False,
            random_seed=None,
            _am_i_a_traffic_vehicle=False
    ):
        """
        This Vehicle Config is different from self.get_config(), and it is used to define which modules to use, and
        module parameters. And self.physics_config defines the physics feature of vehicles, such as length/width
        :param vehicle_config: mostly, vehicle module config
        :param random_seed: int
        """
        super(BaseVehicle, self).__init__(name, random_seed)
        vehicle_config = self.default_config.update(vehicle_config)
        self.update_config(vehicle_config, allow_add_new_key=True)
        assert vehicle_config is not None, "Please specify the vehicle config."
        self.action_space = self.get_action_space_before_init(extra_action_dim=self.config["extra_action_dim"])

        # TODO(pzh): This is a stupid workaround!!!
        self._am_i_a_traffic_vehicle = _am_i_a_traffic_vehicle
        if self._am_i_a_traffic_vehicle:
            self._config["use_render"] = False
            self._config["use_image"] = False
        if self._am_i_a_traffic_vehicle:
            assert not self.config["use_render"]

        self.increment_steering = self.config["increment_steering"]
        self.enable_reverse = self.config["enable_reverse"]
        self.max_speed = self.config["max_speed"]
        self.max_steering = self.config["max_steering"]

        assert self.engine is not None, "Please make sure PGDrive engine is successfully initialized!"

        self.node_path = NodePath("vehicle")

        # color
        color = sns.color_palette("colorblind")
        idx = get_np_random().randint(len(color))
        rand_c = color[idx]
        self.top_down_color = (rand_c[0] * 255, rand_c[1] * 255, rand_c[2] * 255)
        self.panda_color = rand_c

        # create
        self.spawn_place = (0, 0)
        self._add_chassis(self.engine.physics_world)
        self.wheels = self._create_wheel()

        # modules
        self.image_sensors = {}
        self.lidar: Optional[Lidar] = None
        self.side_detector: Optional[SideDetector] = None
        self.lane_line_detector: Optional[LaneLineDetector] = None
        self.routing_localization: Optional[RoutingLocalizationModule] = None
        self.lane: Optional[AbstractLane] = None
        self.lane_index = None
        self.vehicle_panel = VehiclePanel(self.engine) if (self.engine.mode == RENDER_MODE_ONSCREEN) else None

        # state info
        self.throttle_brake = 0.0
        self.steering = 0
        self.last_current_action = deque([(0.0, 0.0), (0.0, 0.0)], maxlen=2)
        self.last_position = self.spawn_place
        self.last_heading_dir = self.heading
        self.dist_to_left_side = None
        self.dist_to_right_side = None

        # collision info render
        self.collision_info_np = self._init_collision_info_render(self.engine)
        self.collision_banners = {}  # to save time
        self.current_banner = None
        self.attach_to_world(self.engine.pbr_render, self.engine.physics_world)

        # step info
        self.out_of_route = None
        self.on_lane = None
        # self.step_info = None
        self._init_step_info()

        # others
        self._add_modules_for_vehicle(self.config["use_render"])
        self.takeover = False
        self._expert_takeover = False
        self.energy_consumption = 0

        # overtake_stat
        self.front_vehicles = set()
        self.back_vehicles = set()

        # color
        color = sns.color_palette("colorblind")
        idx = get_np_random().randint(len(color))
        rand_c = color[idx]
        if am_i_the_special_one:
            rand_c = color[2]  # A pretty green
        self.top_down_color = (rand_c[0] * 255, rand_c[1] * 255, rand_c[2] * 255)

        self._reset_once = False

    def _add_modules_for_vehicle(self, use_render: bool):
        # add self module for training according to config
        vehicle_config = self.config
        self.add_routing_localization(vehicle_config["show_navi_mark"])  # default added

        if self.config["side_detector"]["num_lasers"] > 0:
            self.side_detector = SideDetector(
                self.engine.render, self.config["side_detector"]["num_lasers"],
                self.config["side_detector"]["distance"], self.config["show_side_detector"]
            )

        if self.config["lane_line_detector"]["num_lasers"] > 0:
            self.lane_line_detector = LaneLineDetector(
                self.engine.render, self.config["lane_line_detector"]["num_lasers"],
                self.config["lane_line_detector"]["distance"], self.config["show_lane_line_detector"]
            )

        if not self.config["use_image"]:
            if vehicle_config["lidar"]["num_lasers"] > 0 and vehicle_config["lidar"]["distance"] > 0:
                self.add_lidar(
                    num_lasers=vehicle_config["lidar"]["num_lasers"],
                    distance=vehicle_config["lidar"]["distance"],
                    show_lidar_point=vehicle_config["show_lidar"]
                )
            else:
                import logging
                logging.warning(
                    "You have set the lidar config to: {}, which seems to be invalid!".format(vehicle_config["lidar"])
                )

            if use_render:
                rgb_cam_config = vehicle_config["rgb_cam"]
                assert not self._am_i_a_traffic_vehicle
                rgb_cam = RGBCamera(rgb_cam_config[0], rgb_cam_config[1], self.chassis_np)
                self.add_image_sensor("rgb_cam", rgb_cam)

                mini_map = MiniMap(vehicle_config["mini_map"], self.chassis_np)
                self.add_image_sensor("mini_map", mini_map)
            return

        if vehicle_config["use_image"]:
            assert not self._am_i_a_traffic_vehicle
            # 3 types image observation
            if vehicle_config["image_source"] == "rgb_cam":
                rgb_cam_config = vehicle_config["rgb_cam"]
                rgb_cam = RGBCamera(rgb_cam_config[0], rgb_cam_config[1], self.chassis_np)
                self.add_image_sensor("rgb_cam", rgb_cam)
            elif vehicle_config["image_source"] == "mini_map":
                mini_map = MiniMap(vehicle_config["mini_map"], self.chassis_np)
                self.add_image_sensor("mini_map", mini_map)
            elif vehicle_config["image_source"] == "depth_cam":
                cam_config = vehicle_config["depth_cam"]
                depth_cam = DepthCamera(*cam_config, self.chassis_np, self.engine)
                self.add_image_sensor("depth_cam", depth_cam)
            else:
                raise ValueError("No module named {}".format(vehicle_config["image_source"]))

        # load more sensors for visualization when render, only for beauty...
        if use_render:
            if vehicle_config["image_source"] == "mini_map":
                rgb_cam_config = vehicle_config["rgb_cam"]
                rgb_cam = RGBCamera(rgb_cam_config[0], rgb_cam_config[1], self.chassis_np)
                self.add_image_sensor("rgb_cam", rgb_cam)
            else:
                mini_map = MiniMap(vehicle_config["mini_map"], self.chassis_np)
                self.add_image_sensor("mini_map", mini_map)

    def _init_step_info(self):
        # done info will be initialized every frame
        self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).init_collision_info()
        self.out_of_route = False  # re-route is required if is false
        self.on_lane = True  # on lane surface or not
        # self.step_info = {"reward": 0, "cost": 0}

    def _preprocess_action(self, action):
        if self.config["action_check"]:
            assert self.action_space.contains(action), "Input {} is not compatible with action space {}!".format(
                action, self.action_space
            )

        # protect agent from nan error
        action = safe_clip_for_small_array(action, min_val=self.action_space.low[0], max_val=self.action_space.high[0])
        return action, {'raw_action': (action[0], action[1])}

    def before_step(
            self,
            action,
    ):
        """
        Save info and make decision before action
        """

        assert self._reset_once, "You should reset the base vehicle at least once before controlling it!"

        # init step info to store info before each step
        self._init_step_info()
        action, step_info = self._preprocess_action(action)

        self.last_position = self.position
        self.last_heading_dir = self.heading
        self.last_current_action.append(action)  # the real step of physics world is implemented in taskMgr.step()
        if self.increment_steering:
            self.set_incremental_action(action)
        else:
            self.set_act(action)
        if self.vehicle_panel is not None:
            self.vehicle_panel.renew_2d_car_para_visualization(self)
        return step_info

    def after_step(self, engine=None, detector_mask="WRONG"):
        # lidar
        if self.lidar is not None:
            self.lidar.perceive(
                self.position,
                self.heading_theta,
                self.engine.physics_world.dynamic_world,
                extra_filter_node={self.chassis_np.node()},
                detector_mask=detector_mask
            )
        if self.routing_localization is not None:
            self.lane, self.lane_index, = self.routing_localization.update_navigation_localization(self)
        if self.side_detector is not None:
            self.side_detector.perceive(self.position, self.heading_theta, self.engine.physics_world.static_world)
        if self.lane_line_detector is not None:
            self.lane_line_detector.perceive(self.position, self.heading_theta, self.engine.physics_world.static_world)
        self._state_check()

        # TODO(pzh): This is a workaround to discriminate traffic / ego vehicle.
        if len(self.routing_localization.checkpoints) > 0:
            self.update_dist_to_left_right()
            step_energy, episode_energy = self._update_energy_consumption()
            self.out_of_route = self._out_of_route()
        else:
            step_energy = 0.0
            episode_energy = 0.0

        step_info = self._update_overtake_stat()
        step_info.update(
            {
                "velocity": float(self.speed),
                "steering": float(self.steering),
                "acceleration": float(self.throttle_brake),
                "step_energy": step_energy,
                "episode_energy": episode_energy
            }
        )
        return step_info

    def _out_of_route(self):
        left, right = self._dist_to_route_left_right()
        return True if right < 0 or left < 0 else False

    def _update_energy_consumption(self):
        """
        The calculation method is from
        https://www.researchgate.net/publication/262182035_Reduction_of_Fuel_Consumption_and_Exhaust_Pollutant_Using_Intelligent_Transport_System
        default: 3rd gear, try to use ae^bx to fit it, dp: (90, 8), (130, 12)
        :return: None
        """
        distance = norm(*(self.last_position - self.position)) / 1000  # km
        step_energy = 3.25 * math.pow(np.e, 0.01 * self.speed) * distance / 100
        # step_energy is in Liter, we return mL
        step_energy = step_energy * 1000
        self.energy_consumption += step_energy  # L/100 km
        return step_energy, self.energy_consumption

    def reset(self, map: BaseMap, pos: np.ndarray = None, heading: float = 0.0):
        """
        pos is a 2-d array, and heading is a float (unit degree)
        if pos is not None, vehicle will be reset to the position
        else, vehicle will be reset to spawn place
        """
        # TODO(pzh): Who provide a heading here?? If this is a traffic vehicle, then the pos and heading should be
        #  determined by the traffic manager!!!!!!!!

        if pos is None:
            lane = map.road_network.get_lane(self.config["spawn_lane_index"])
            pos = lane.position(self.config["spawn_longitude"], self.config["spawn_lateral"])
            heading = np.rad2deg(lane.heading_at(self.config["spawn_longitude"]))
            self.spawn_place = pos
        heading = -np.deg2rad(heading) - np.pi / 2
        self.set_static(False)
        self.chassis_np.setPos(panda_position(Vec3(*pos, 1)))
        self.chassis_np.setQuat(LQuaternionf(math.cos(heading / 2), 0, 0, math.sin(heading / 2)))

        # TODO(pzh): This is a extremely stupid workaround!! Remove this!
        if not self._am_i_a_traffic_vehicle:
            self.update_map_info(map)

        self.chassis_np.node().clearForces()
        self.chassis_np.node().setLinearVelocity(Vec3(0, 0, 0))
        self.chassis_np.node().setAngularVelocity(Vec3(0, 0, 0))
        self.system.resetSuspension()
        # np.testing.assert_almost_equal(self.position, pos, decimal=4)

        # done info
        self._init_step_info()

        # other info
        self.throttle_brake = 0.0
        self.steering = 0
        self.last_current_action = deque([(0.0, 0.0), (0.0, 0.0)], maxlen=2)
        self.last_position = self.spawn_place
        self.last_heading_dir = self.heading

        self.update_dist_to_left_right()
        self.takeover = False
        self.energy_consumption = 0

        # overtake_stat
        # TODO: Remove this!! A single instance of the vehicle should not access its context!!!
        self.front_vehicles = set()
        self.back_vehicles = set()

        # TODO: This should be put into the render-object of this vehicle!
        # for render
        if self.vehicle_panel is not None:
            self.vehicle_panel.renew_2d_car_para_visualization(self)

        if "depth_cam" in self.image_sensors and self.image_sensors["depth_cam"].view_ground:
            for block in map.blocks:
                block.node_path.hide(CamMask.DepthCam)

        assert self.routing_localization
        # Please note that if you respawn agent to some new place and might have a new destination,
        # you should reset the routing localization too! Via: vehicle.routing_localization.set_route or
        # vehicle.update

        self._reset_once = True

    """------------------------------------------- act -------------------------------------------------"""

    def set_act(self, action):
        steering = action[0]
        self.throttle_brake = action[1]
        self.steering = steering
        self.system.setSteeringValue(self.steering * self.max_steering, 0)
        self.system.setSteeringValue(self.steering * self.max_steering, 1)
        self._apply_throttle_brake(action[1])

    def set_incremental_action(self, action: np.ndarray):
        self.throttle_brake = action[1]
        self.steering += action[0] * self.STEERING_INCREMENT
        self.steering = clip(self.steering, -1, 1)
        steering = self.steering * self.max_steering
        self.system.setSteeringValue(steering, 0)
        self.system.setSteeringValue(steering, 1)
        self._apply_throttle_brake(action[1])

    def _apply_throttle_brake(self, throttle_brake):
        max_engine_force = self.config["max_engine_force"]
        max_brake_force = self.config["max_brake_force"]
        for wheel_index in range(4):
            if throttle_brake >= 0:
                self.system.setBrake(2.0, wheel_index)
                if self.speed > self.max_speed:
                    self.system.applyEngineForce(0.0, wheel_index)
                else:
                    self.system.applyEngineForce(max_engine_force * throttle_brake, wheel_index)
            else:
                if self.enable_reverse:
                    self.system.applyEngineForce(max_engine_force * throttle_brake, wheel_index)
                    self.system.setBrake(0, wheel_index)
                else:
                    self.system.applyEngineForce(0.0, wheel_index)
                    self.system.setBrake(abs(throttle_brake) * max_brake_force, wheel_index)

    """---------------------------------------- vehicle info ----------------------------------------------"""

    def update_dist_to_left_right(self):
        self.dist_to_left_side, self.dist_to_right_side = self._dist_to_route_left_right()

    def _dist_to_route_left_right(self):
        # TODO(pzh): This function is really stupid! Why we maintain such information in BaseVehicle???
        if not self.routing_localization.current_ref_lanes:
            # TODO(pzh): This is a workaround! It looks like this vehicle is a traffic vehicle!! So we don't maintain
            #  this information!
            return np.nan, np.nan
        current_reference_lane = self.routing_localization.current_ref_lanes[0]
        _, lateral_to_reference = current_reference_lane.local_coordinates(self.position)
        lateral_to_left = lateral_to_reference + self.routing_localization.get_current_lane_width() / 2
        lateral_to_right = self.routing_localization.get_current_lateral_range(
            self.position, self.engine
        ) - lateral_to_left
        return lateral_to_left, lateral_to_right

    @property
    def position(self):
        return pgdrive_position(self.chassis_np.getPos())

    @property
    def speed(self):
        """
        km/h
        """
        velocity = self.chassis_np.node().get_linear_velocity()
        speed = norm(velocity[0], velocity[1]) * 3.6
        return clip(speed, 0.0, 100000.0)

    @property
    def heading(self):
        real_heading = self.heading_theta
        # heading = np.array([math.cos(real_heading), math.sin(real_heading)])
        heading = Vector((math.cos(real_heading), math.sin(real_heading)))
        return heading

    @property
    def heading_theta(self):
        """
        Get the heading theta of vehicle, unit [rad]
        :return:  heading in rad
        """
        return (pgdrive_heading(self.chassis_np.getH()) - 90) / 180 * math.pi

    @property
    def velocity(self) -> np.ndarray:
        return self.speed * self.velocity_direction

    @property
    def velocity_direction(self):
        direction = self.system.getForwardVector()
        return np.asarray([direction[0], -direction[1]])

    @property
    def current_road(self):
        return Road(*self.lane_index[0:-1])

    """---------------------------------------- some math tool ----------------------------------------------"""

    def heading_diff(self, target_lane):
        lateral = None
        if isinstance(target_lane, StraightLane):
            lateral = np.asarray(get_vertical_vector(target_lane.end - target_lane.start)[1])
        elif isinstance(target_lane, CircularLane):
            if target_lane.direction == -1:
                lateral = self.position - target_lane.center
            else:
                lateral = target_lane.center - self.position
        elif isinstance(target_lane, WayPointLane):
            lane_segment = target_lane.segment(target_lane.local_coordinates(self.position)[0])
            lateral = lane_segment["lateral_direction"]

        lateral_norm = norm(lateral[0], lateral[1])
        forward_direction = self.heading
        # print(f"Old forward direction: {self.forward_direction}, new heading {self.heading}")
        forward_direction_norm = norm(forward_direction[0], forward_direction[1])
        if not lateral_norm * forward_direction_norm:
            return 0
        cos = (
                (forward_direction[0] * lateral[0] + forward_direction[1] * lateral[1]) /
                (lateral_norm * forward_direction_norm)
        )
        # return cos
        # Normalize to 0, 1
        return clip(cos, -1.0, 1.0) / 2 + 0.5

    def projection(self, vector):
        # Projected to the heading of vehicle
        # forward_vector = self.vehicle.get_forward_vector()
        # forward_old = (forward_vector[0], -forward_vector[1])

        forward = self.heading

        # print(f"[projection] Old forward {forward_old}, new heading {forward}")

        norm_velocity = norm(forward[0], forward[1]) + 1e-6
        project_on_heading = (vector[0] * forward[0] + vector[1] * forward[1]) / norm_velocity

        side_direction = get_vertical_vector(forward)[1]
        side_norm = norm(side_direction[0], side_direction[1]) + 1e-6
        project_on_side = (vector[0] * side_direction[0] + vector[1] * side_direction[1]) / side_norm
        return project_on_heading, project_on_side

    def lane_distance_to(self, vehicle, lane: AbstractLane = None) -> float:
        assert self.routing_localization is not None, "a routing and localization module should be added " \
                                                      "to interact with other vehicles"
        if not vehicle:
            return np.nan
        if not lane:
            lane = self.lane
        return lane.local_coordinates(vehicle.position)[0] - lane.local_coordinates(self.position)[0]

    """-------------------------------------- for vehicle making ------------------------------------------"""

    def _add_chassis(self, physics_world: PhysicsWorld):
        para = self.get_config()

        # TODO(pzh): This part access many config!!! Would this part affect the creation of traffic vehicle???
        self.LENGTH = self.config["vehicle_length"]
        self.WIDTH = self.config["vehicle_width"]

        chassis = BaseVehicleNode(BodyName.Base_vehicle, self)
        chassis.setIntoCollideMask(BitMask32.bit(CollisionGroup.EgoVehicle))
        chassis_shape = BulletBoxShape(Vec3(self.WIDTH / 2, self.LENGTH / 2, para[Parameter.vehicle_height] / 2))
        ts = TransformState.makePos(Vec3(0, 0, para[Parameter.chassis_height] * 2))
        chassis.addShape(chassis_shape, ts)
        heading = np.deg2rad(-para[Parameter.heading] - 90)  # pzh: like this one!
        chassis.setMass(para[Parameter.mass])
        self.chassis_np = self.node_path.attachNewNode(chassis)

        # not random spawn now
        self.chassis_np.setPos(Vec3(*self.spawn_place, 1))
        self.chassis_np.setQuat(LQuaternionf(math.cos(heading / 2), 0, 0, math.sin(heading / 2)))
        chassis.setDeactivationEnabled(False)
        chassis.notifyCollisions(True)  # advance collision check, do callback in pg_collision_callback
        self.dynamic_nodes.append(chassis)

        chassis_beneath = BulletGhostNode(BodyName.Base_vehicle_beneath)
        chassis_beneath.setIntoCollideMask(BitMask32.bit(CollisionGroup.EgoVehicleBeneath))
        chassis_beneath.addShape(chassis_shape)
        self.chassis_beneath_np = self.chassis_np.attachNewNode(chassis_beneath)
        self.dynamic_nodes.append(chassis_beneath)

        self.system = BulletVehicle(physics_world.dynamic_world, chassis)
        self.system.setCoordinateSystem(ZUp)
        self.dynamic_nodes.append(self.system)  # detach chassis will also detach system, so a waring will generate

        if self.render:
            if self.MODEL is None:

                # TODO(pzh): We should put ego into default config!
                model_details = self.config["model_details"] or MODEL_TYPES["ego"]

                model_path = model_details["model"]["path"]
                self.MODEL = self.loader.loadModel(AssetLoader.file_path(model_path))

                # Set heading
                if model_details["model"].get("heading", None) is not None:
                    # TODO(pzh): This is a workaround! We should change all +90, -pi/2 ... after refactoring coord.sys.
                    self.MODEL.setH(model_details["model"]["heading"] + 90)
                else:
                    self.MODEL.setH(para[Parameter.vehicle_vis_h])

                # Set scale
                if model_details["model"].get("scale", None) is not None:
                    self.MODEL.set_scale(model_details["model"]["scale"])
                else:
                    self.MODEL.set_scale(para[Parameter.vehicle_vis_scale])

                # Set offset (only applicable to traffic vehicle)
                if model_details["model"].get("offset", None) is not None:
                    self.MODEL.setPos(model_details["model"]["offset"])
                else:
                    # TODO(pzh): What the hell is this? Should we add this config into traffic vehicles too?
                    self.MODEL.setZ(para[Parameter.vehicle_vis_z])
                    self.MODEL.setY(para[Parameter.vehicle_vis_y])

            self.MODEL.instanceTo(self.chassis_np)

            if self.config["random_color"]:
                # TODO(pzh): We can randomized the color of traffic vehicle!
                material = Material()
                material.setBaseColor(
                    (
                        self.panda_color[0] * self.MATERIAL_COLOR_COEFF,
                        self.panda_color[1] * self.MATERIAL_COLOR_COEFF,
                        self.panda_color[2] * self.MATERIAL_COLOR_COEFF, 0.2
                    )
                )
                material.setMetallic(self.MATERIAL_METAL_COEFF)
                material.setSpecular(self.MATERIAL_SPECULAR_COLOR)
                material.setRefractiveIndex(1.5)
                material.setRoughness(self.MATERIAL_ROUGHNESS)
                material.setShininess(self.MATERIAL_SHININESS)
                material.setTwoside(False)
                self.chassis_np.setMaterial(material, True)

    def _create_wheel(self):
        para = self.get_config()
        f_l = para[Parameter.front_tire_longitude]
        r_l = -para[Parameter.rear_tire_longitude]
        lateral = para[Parameter.tire_lateral]
        axis_height = para[Parameter.tire_radius] + 0.05
        radius = para[Parameter.tire_radius]
        wheels = []
        for k, pos in enumerate([Vec3(lateral, f_l, axis_height), Vec3(-lateral, f_l, axis_height),
                                 Vec3(lateral, r_l, axis_height), Vec3(-lateral, r_l, axis_height)]):
            wheel = self._add_wheel(pos, radius, True if k < 2 else False, True if k == 0 or k == 2 else False)
            wheels.append(wheel)
        return wheels

    def _add_wheel(self, pos: Vec3, radius: float, front: bool, left):
        wheel_np = self.node_path.attachNewNode("wheel")
        if self.render:
            # TODO something wrong with the wheel render
            model_path = 'models/yugo/yugotireR.egg' if left else 'models/yugo/yugotireL.egg'
            wheel_model = self.loader.loadModel(AssetLoader.file_path(model_path))
            wheel_model.reparentTo(wheel_np)
            wheel_model.set_scale(1.4, radius / 0.25, radius / 0.25)
        wheel = self.system.create_wheel()
        wheel.setNode(wheel_np.node())
        wheel.setChassisConnectionPointCs(pos)
        wheel.setFrontWheel(front)
        wheel.setWheelDirectionCs(Vec3(0, 0, -1))
        wheel.setWheelAxleCs(Vec3(1, 0, 0))

        # TODO add them to Config in the future
        wheel.setWheelRadius(radius)
        wheel.setMaxSuspensionTravelCm(40)
        wheel.setSuspensionStiffness(30)
        wheel.setWheelsDampingRelaxation(4.8)
        wheel.setWheelsDampingCompression(1.2)
        wheel.setFrictionSlip(self.config["wheel_friction"])
        wheel.setRollInfluence(1.5)
        return wheel

    def add_image_sensor(self, name: str, sensor: ImageBuffer):
        self.image_sensors[name] = sensor

    def add_lidar(self, num_lasers=240, distance=50, show_lidar_point=False):
        assert num_lasers > 0
        assert distance > 0
        self.lidar = Lidar(self.engine.render, num_lasers, distance, show_lidar_point)

    def add_routing_localization(self, show_navi_mark: bool = False):
        config = self.config
        self.routing_localization = RoutingLocalizationModule(
            self.engine,
            show_navi_mark=show_navi_mark,
            random_navi_mark_color=config["random_navi_mark_color"],
            show_dest_mark=config["show_dest_mark"],
            show_line_to_dest=config["show_line_to_dest"]
        )

    def update_map_info(self, map):
        """
        Update map info after reset()
        :param map: new map
        :return: None
        """
        possible_lanes = ray_localization(
            np.array(self.heading.tolist()), np.array(self.spawn_place), self.engine, return_all_result=True
        )
        possible_lane_indexes = [lane_index for lane, lane_index, dist in possible_lanes]
        try:
            idx = possible_lane_indexes.index(self.config["spawn_lane_index"])
        except ValueError:
            lane, new_l_index = possible_lanes[0][:-1]
        else:
            lane, new_l_index = possible_lanes[idx][:-1]
        dest = self.config["destination_lane_index"]
        self.routing_localization.update(
            map, current_lane_index=new_l_index, final_road_node=dest[1] if dest is not None else None
        )
        assert lane is not None, "spawn place is not on road!"
        self.lane_index = new_l_index
        self.lane = lane

    def _state_check(self):
        """
        Check States and filter to update info
        """
        result_1 = self.engine.physics_world.static_world.contactTest(self.chassis_beneath_np.node(), True)
        result_2 = self.engine.physics_world.dynamic_world.contactTest(self.chassis_beneath_np.node(), True)
        contacts = set()
        for contact in result_1.getContacts() + result_2.getContacts():
            node0 = contact.getNode0()
            node1 = contact.getNode1()
            name = [node0.getName(), node1.getName()]
            name.remove(BodyName.Base_vehicle_beneath)
            if name[0] == "Ground" or name[0] == BodyName.Lane:
                continue
            if name[0] == BodyName.Sidewalk:
                self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).crash_sidewalk = True
            elif name[0] == BodyName.White_continuous_line:
                self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).on_white_continuous_line = True
            elif name[0] == BodyName.Yellow_continuous_line:
                self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).on_yellow_continuous_line = True
            elif name[0] == BodyName.Broken_line:
                self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).on_broken_line = True
            contacts.add(name[0])
        if self.render:
            self.render_collision_info(contacts)

    @staticmethod
    def _init_collision_info_render(engine):
        if engine.mode == "onscreen":
            info_np = NodePath("Collision info nodepath")
            info_np.reparentTo(engine.aspect2d)
        else:
            info_np = None
        return info_np

    def render_collision_info(self, contacts):
        contacts = sorted(list(contacts), key=lambda c: COLLISION_INFO_COLOR[COLOR[c]][0])
        text = contacts[0] if len(contacts) != 0 else None
        if text is None:
            text = "Normal" if time.time() - self.engine._episode_start_time > 10 else "Press H to see help message"
            self.render_banner(text, COLLISION_INFO_COLOR["green"][1])
        else:
            if text == BodyName.Base_vehicle_beneath:
                text = BodyName.Traffic_vehicle
            self.render_banner(text, COLLISION_INFO_COLOR[COLOR[text]][1])

    def render_banner(self, text, color=COLLISION_INFO_COLOR["green"][1]):
        """
        Render the banner in the left bottom corner.
        """
        if self.collision_info_np is None:
            return
        if self.current_banner is not None:
            self.current_banner.detachNode()
        if text in self.collision_banners:
            self.collision_banners[text].reparentTo(self.collision_info_np)
            self.current_banner = self.collision_banners[text]
        else:
            new_banner = NodePath(TextNode("collision_info:{}".format(text)))
            self.collision_banners[text] = new_banner
            text_node = new_banner.node()
            text_node.setCardColor(color)
            text_node.setText(text)
            text_node.setCardActual(-5 * self.engine.w_scale, 5.1 * self.engine.w_scale, -0.3, 1)
            text_node.setCardDecal(True)
            text_node.setTextColor(1, 1, 1, 1)
            text_node.setAlign(TextNode.A_center)
            new_banner.setScale(0.05)
            new_banner.setPos(-0.75 * self.engine.w_scale, 0, -0.8 * self.engine.h_scale)
            new_banner.reparentTo(self.collision_info_np)
            self.current_banner = new_banner

    def destroy(self):
        self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).destroy()
        if self.chassis_np.node() in self.dynamic_nodes:
            self.dynamic_nodes.remove(self.chassis_np.node())
        super(BaseVehicle, self).destroy()
        self.engine.physics_world.dynamic_world.clearContactAddedCallback()
        self.routing_localization.destroy()
        self.routing_localization = None

        if self.side_detector is not None:
            self.side_detector.destroy()

        if self.lane_line_detector is not None:
            self.lane_line_detector.destroy()

        self.side_detector = None
        self.lane_line_detector = None

        if self.lidar is not None:
            self.lidar.destroy()
            self.lidar = None
        if len(self.image_sensors) != 0:
            for sensor in self.image_sensors.values():
                sensor.destroy()
        self.image_sensors = None
        if self.vehicle_panel is not None:
            self.vehicle_panel.destroy()

    def set_position(self, position, height=0.4):
        """
        Should only be called when restore traffic from episode data
        :param position: 2d array or list
        :return: None
        """
        self.chassis_np.setPos(panda_position(position, height))

    def set_heading(self, heading_theta) -> None:
        """
        Should only be called when restore traffic from episode data
        :param heading_theta: float in rad
        :return: None
        """
        self.chassis_np.setH((panda_heading(heading_theta) * 180 / np.pi) - 90)

    def get_state(self):
        final_road = self.routing_localization.final_road
        return {
            "heading": self.heading_theta,
            "position": self.position.tolist(),
            "done": self.crash_vehicle or self.out_of_route or self.crash_sidewalk or not self.on_lane,
            "speed": self.speed,
            "spawn_road": self.config["spawn_lane_index"][:-1],
            "destination": (final_road.start_node, final_road.end_node)
        }

    def set_state(self, state: dict):
        self.set_heading(state["heading"])
        self.set_position(state["position"])
        self._replay_done = state["done"]
        self.set_position(state["position"], height=0.28)

    def _update_overtake_stat(self):
        if self.config["overtake_stat"]:
            surrounding_vs = self.lidar.get_surrounding_vehicles()
            routing = self.routing_localization
            ckpt_idx = routing._target_checkpoints_index
            for surrounding_v in surrounding_vs:
                if surrounding_v.lane_index[:-1] == (routing.checkpoints[ckpt_idx[0]], routing.checkpoints[ckpt_idx[1]
                ]):
                    if self.lane.local_coordinates(self.position)[0] - \
                            self.lane.local_coordinates(surrounding_v.position)[0] < 0:
                        self.front_vehicles.add(surrounding_v)
                        if surrounding_v in self.back_vehicles:
                            self.back_vehicles.remove(surrounding_v)
                    else:
                        self.back_vehicles.add(surrounding_v)
        return {"overtake_vehicle_num": self.get_overtake_num()}

    def get_overtake_num(self):
        return len(self.front_vehicles.intersection(self.back_vehicles))

    @classmethod
    def get_action_space_before_init(cls, extra_action_dim: int = 0):
        return gym.spaces.Box(-1.0, 1.0, shape=(2 + extra_action_dim,), dtype=np.float32)

    def remove_display_region(self):
        if self.render:
            self.vehicle_panel.remove_display_region()
            self.vehicle_panel.buffer.set_active(False)
            self.collision_info_np.detachNode()
            self.routing_localization._arrow_node_path.detachNode()
        for sensor in self.image_sensors.values():
            sensor.remove_display_region()
            sensor.buffer.set_active(False)

    def add_to_display(self):
        if self.render:
            self.vehicle_panel.add_to_display(self.vehicle_panel.default_region)
            self.vehicle_panel.buffer.set_active(True)
            self.collision_info_np.reparentTo(self.engine.aspect2d)
            self.routing_localization._arrow_node_path.reparentTo(self.engine.aspect2d)
        for sensor in self.image_sensors.values():
            sensor.add_to_display(sensor.default_region)
            sensor.buffer.set_active(True)

    def __del__(self):
        super(BaseVehicle, self).__del__()
        self.lidar = None
        self.mini_map = None
        self.rgb_cam = None
        self.routing_localization = None
        self.wheels = None

    @property
    def arrive_destination(self):
        long, lat = self.routing_localization.final_lane.local_coordinates(self.position)
        flag = (
                       self.routing_localization.final_lane.length - 5 < long < self.routing_localization.final_lane.length + 5
               ) and (
                       self.routing_localization.get_current_lane_width() / 2 >= lat >=
                       (0.5 - self.routing_localization.get_current_lane_num()) *
                       self.routing_localization.get_current_lane_width()
               )
        return flag

    @property
    def crash_vehicle(self):
        return self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).crash_vehicle

    @property
    def crash_object(self):
        return self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).crash_object

    @property
    def crash_sidewalk(self):
        return self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).crash_sidewalk

    @property
    def on_yellow_continuous_line(self):
        return self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).on_yellow_continuous_line

    @property
    def on_white_continuous_line(self):
        return self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).on_white_continuous_line

    @property
    def on_broken_line(self):
        return self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).on_broken_line

    def set_static(self, flag):
        self.chassis_np.node().setStatic(flag)

    @property
    def crash_building(self):
        return self.chassis_np.node().getPythonTag(BodyName.Base_vehicle).crash_building

    @property
    def reference_lanes(self):
        return self.routing_localization.current_ref_lanes

    def set_wheel_friction(self, new_friction):
        raise DeprecationWarning("Bug exists here")
        for wheel in self.wheels:
            wheel.setFrictionSlip(new_friction)

    @property
    def overspeed(self):
        return True if self.lane.speed_limit < self.speed else False

    @property
    def replay_done(self):
        return self._replay_done if hasattr(self, "_replay_done") else (
                self.crash_building or self.crash_vehicle or
                # self.on_white_continuous_line or
                self.on_yellow_continuous_line
        )

    def to_dict(self, origin_vehicle: "Vehicle" = None, observe_intentions: bool = True) -> dict:
        d = {
            'presence': 1,
            'x': self.position[0],
            'y': self.position[1],
            'vx': self.velocity[0],
            'vy': self.velocity[1],
            'cos_h': self.direction[0],
            'sin_h': self.direction[1],
            # 'cos_d': self.destination_direction[0],
            # 'sin_d': self.destination_direction[1]
        }
        if not observe_intentions:
            d["cos_d"] = d["sin_d"] = 0
        if origin_vehicle:
            origin_dict = origin_vehicle.to_dict()
            for key in ['x', 'y', 'vx', 'vy']:
                d[key] -= origin_dict[key]
        return d

    @property
    def direction(self) -> np.ndarray:
        return np.array([math.cos(self.heading_theta), math.sin(self.heading_theta)])

    @property
    def out_of_road(self):
        # TODO(pzh): This function is very stupid!
        p = get_engine().policy_manager.get_policy(self.name)
        ret = not p.lane.on_lane(self.position, margin=2)
        return ret

    def need_remove(self):

        # TODO(pzh): The following is copied from base vehicle. What should we do if a base vehicle should be removed?

        # if self._initial_state is not None:
        #     return False
        # else:
        #     self.vehicle_node.clearTag(BodyName.Traffic_vehicle)
        #     self.node_path.removeNode()
        #     print("The vehicle is removed!")
        #     return True

        pass
