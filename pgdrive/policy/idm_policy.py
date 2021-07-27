from typing import Tuple, Union, List

import numpy as np

import pgdrive.utils.math_utils as utils
from pgdrive.constants import Route, LaneIndex
from pgdrive.policy.base_policy import BasePolicy
from pgdrive.scene_creator.highway_vehicle.controller import ControlledVehicle
from pgdrive.scene_creator.highway_vehicle.kinematics import Vehicle
from pgdrive.scene_creator.object.static_object import StaticObject
from pgdrive.scene_managers.traffic_manager import TrafficManager
from pgdrive.utils.math_utils import clip


class IDMPolicy(BasePolicy):
    """
    A vehicle using both a longitudinal and a lateral decision policies.

    - Longitudinal: the IDM model computes an acceleration given the preceding vehicle's distance and speed.
    - Lateral: the MOBIL model decides when to change lane by maximizing the acceleration of nearby vehicles.
    """

    # Longitudinal policy parameters
    ACC_MAX = 20.0  # [m/s2]
    """Maximum acceleration."""

    COMFORT_ACC_MAX = 10.0  # [m/s2]
    """Desired maximum acceleration."""

    COMFORT_ACC_MIN = -10.0  # [m/s2]
    """Desired maximum deceleration."""

    DISTANCE_WANTED = 5.0 + ControlledVehicle.LENGTH  # [m]
    """Desired jam distance to the front vehicle."""

    TIME_WANTED = 1.5  # [s]
    """Desired time gap to the front vehicle."""

    DELTA = 2.0  # []
    """Exponent of the velocity term."""

    # Lateral policy parameters
    POLITENESS = 0.  # in [0, 1]
    LANE_CHANGE_MIN_ACC_GAIN = 0.2  # [m/s2]
    LANE_CHANGE_MAX_BRAKING_IMPOSED = 2.0  # [m/s2]
    LANE_CHANGE_DELAY = 1.0  # [s]

    def __init__(
            self,
            traffic_mgr: TrafficManager,
            position: List,
            delay_time: float,
            heading: float = 0,
            speed: float = 0,
            target_lane_index: int = None,
            target_speed: float = None,
            route: Route = None,
            enable_lane_change: bool = True,
            np_random: np.random.RandomState = None,
    ):
        super().__init__(
            traffic_mgr, position, heading, speed, target_lane_index, target_speed, route, np_random=np_random
        )
        self.enable_lane_change = enable_lane_change
        self.delay_time = delay_time

    # def randomize_behavior(self):
    #     pass
    #
    # @classmethod
    # def create_from(cls, vehicle: ControlledVehicle) -> "IDMVehicle":
    #     """
    #     Create a new vehicle from an existing one.
    #
    #     The vehicle dynamics and target dynamics are copied, other properties are default.
    #
    #     :param vehicle: a vehicle
    #     :return: a new vehicle at the same dynamical state
    #     """
    #     v = cls(
    #         vehicle.traffic_mgr,
    #         vehicle.position,
    #         heading=vehicle.heading,
    #         speed=vehicle.speed,
    #         target_lane_index=vehicle.target_lane_index,
    #         target_speed=vehicle.target_speed,
    #         route=vehicle.route,
    #         timer=getattr(vehicle, 'timer', None),
    #         np_random=vehicle.np_random
    #     )
    #     return v

    def act(self, vehicle):
        """
        Execute an action.

        For now, no action is supported because the vehicle takes all decisions
        of acceleration and lane changes on its own, based on the IDM and MOBIL models.
        """
        if vehicle.crashed:
            return
        action = {}
        front_vehicle, rear_vehicle = self.traffic_mgr.neighbour_vehicles(self)
        # Lateral: MOBIL
        self.follow_road()
        if self.enable_lane_change:
            self.change_lane_policy()
        action['steering'] = self.steering_control(self.target_lane_index)
        action['steering'] = clip(action['steering'], -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)

        # Longitudinal: IDM
        action['acceleration'] = self.acceleration(
            ego_vehicle=vehicle, front_vehicle=front_vehicle, rear_vehicle=rear_vehicle
        )
        # action['acceleration'] = self.recover_from_stop(action['acceleration'])
        action['acceleration'] = clip(action['acceleration'], -self.ACC_MAX, self.ACC_MAX)
        # Vehicle.act(self, action)  # Skip ControlledVehicle.act(), or the command will be override.
        vehicle.act(self, action)

    # def step(self, dt: float):
    #     """
    #     Step the simulation.
    #
    #     Increases a timer used for decision policies, and step the vehicle dynamics.
    #
    #     :param dt: timestep
    #     """
    #     self.delay_time += dt
    #     if self.action['acceleration'] < 0 and self.speed <= 0:
    #         self.action['acceleration'] = -self.speed / dt
    #     super().step(dt)

    def acceleration(
            self, ego_vehicle: ControlledVehicle, front_vehicle: Vehicle = None, rear_vehicle: Vehicle = None
    ) -> float:
        """
        Compute an acceleration command with the Intelligent Driver Model.

        The acceleration is chosen so as to:
        - reach a target speed;
        - maintain a minimum safety distance (and safety time) w.r.t the front vehicle.

        :param ego_vehicle: the vehicle whose desired acceleration is to be computed. It does not have to be an
                            IDM vehicle, which is why this method is a class method. This allows an IDM vehicle to
                            reason about other vehicles behaviors even though they may not IDMs.
        :param front_vehicle: the vehicle preceding the ego-vehicle
        :param rear_vehicle: the vehicle following the ego-vehicle
        :return: the acceleration command for the ego-vehicle [m/s2]
        """
        if not ego_vehicle or isinstance(ego_vehicle, StaticObject):
            return 0
        ego_target_speed = utils.not_zero(getattr(ego_vehicle, "target_speed", 0))
        acceleration = self.COMFORT_ACC_MAX * (1 - np.power(max(ego_vehicle.speed, 0) / ego_target_speed, self.DELTA))

        if front_vehicle:
            d = ego_vehicle.lane_distance_to(front_vehicle)
            acceleration -= self.COMFORT_ACC_MAX * \
                            np.power(self.desired_gap(ego_vehicle, front_vehicle) / utils.not_zero(d), 2)
        if acceleration < 0 and self.speed < 0:
            acceleration = -self.speed / 0.2
        return acceleration

    def desired_gap(self, ego_vehicle: Vehicle, front_vehicle: Vehicle = None) -> float:
        """
        Compute the desired distance between a vehicle and its leading vehicle.

        :param ego_vehicle: the vehicle being controlled
        :param front_vehicle: its leading vehicle
        :return: the desired distance between the two [m]
        """
        d0 = self.DISTANCE_WANTED
        tau = self.TIME_WANTED
        ab = -self.COMFORT_ACC_MAX * self.COMFORT_ACC_MIN
        dv = ego_vehicle.speed - front_vehicle.speed
        d_star = d0 + ego_vehicle.speed * tau + ego_vehicle.speed * dv / (2 * np.sqrt(ab))
        return d_star

    def maximum_speed(self, front_vehicle: Vehicle = None) -> Tuple[float, float]:
        """
        Compute the maximum allowed speed to avoid Inevitable Collision States.

        Assume the front vehicle is going to brake at full deceleration and that
        it will be noticed after a given delay, and compute the maximum speed
        which allows the ego-vehicle to brake enough to avoid the collision.

        :param front_vehicle: the preceding vehicle
        :return: the maximum allowed speed, and suggested acceleration
        """
        if not front_vehicle:
            return self.target_speed
        d0 = self.DISTANCE_WANTED
        a0 = self.COMFORT_ACC_MIN
        a1 = self.COMFORT_ACC_MIN
        tau = self.TIME_WANTED
        d = max(self.lane_distance_to(front_vehicle) - self.LENGTH / 2 - front_vehicle.LENGTH / 2 - d0, 0)
        v1_0 = front_vehicle.speed
        delta = 4 * (a0 * a1 * tau) ** 2 + 8 * a0 * (a1 ** 2) * d + 4 * a0 * a1 * v1_0 ** 2
        v_max = -a0 * tau + np.sqrt(delta) / (2 * a1)

        # Speed control
        self.target_speed = min(self.maximum_speed(front_vehicle), self.target_speed)
        acceleration = self.speed_control(self.target_speed)

        return v_max, acceleration

    def change_lane_policy(self) -> None:
        """
        Decide when to change lane.

        Based on:
        - frequency;
        - closeness of the target lane;
        - MOBIL model.
        """
        # If a lane change already ongoing
        if self.lane_index != self.target_lane_index:
            # If we are on correct route but bad lane: abort it if someone else is already changing into the same lane
            if self.lane_index[:2] == self.target_lane_index[:2]:
                for v in self.traffic_mgr.vehicles:
                    if v is not self \
                            and v.lane_index != self.target_lane_index \
                            and isinstance(v, ControlledVehicle) \
                            and v.target_lane_index == self.target_lane_index:
                        d = self.lane_distance_to(v)
                        d_star = self.desired_gap(self, v)
                        if 0 < d < d_star:
                            self.target_lane_index = self.lane_index
                            break
            return

        # else, at a given frequency,
        if not utils.do_every(self.LANE_CHANGE_DELAY, self.delay_time):
            return
        self.delay_time = 0

        # decide to make a lane change
        for lane_index in self.traffic_mgr.current_map.road_network.side_lanes(self.lane_index):
            # Is the candidate lane close enough?
            if not self.traffic_mgr.current_map.road_network.get_lane(lane_index).is_reachable_from(self.position):
                continue
            # Does the MOBIL model recommend a lane change?
            if self.mobil(lane_index):
                self.target_lane_index = lane_index

    def mobil(self, lane_index: LaneIndex) -> bool:
        """
        MOBIL lane change model: Minimizing Overall Braking Induced by a Lane change

            The vehicle should change lane only if:
            - after changing it (and/or following vehicles) can accelerate more;
            - it doesn't impose an unsafe braking on its new following vehicle.

        :param lane_index: the candidate lane for the change
        :return: whether the lane change should be performed
        """
        # Is the maneuver unsafe for the new following vehicle?
        new_preceding, new_following = self.traffic_mgr.neighbour_vehicles(self, lane_index)
        new_following_a = self.acceleration(ego_vehicle=new_following, front_vehicle=new_preceding)
        new_following_pred_a = self.acceleration(ego_vehicle=new_following, front_vehicle=self)
        if new_following_pred_a < -self.LANE_CHANGE_MAX_BRAKING_IMPOSED:
            return False

        # Do I have a planned route for a specific lane which is safe for me to access?
        old_preceding, old_following = self.traffic_mgr.neighbour_vehicles(self)
        self_pred_a = self.acceleration(ego_vehicle=self, front_vehicle=new_preceding)
        if self.route and self.route[0][2]:
            # Wrong direction
            if np.sign(lane_index[2] - self.target_lane_index[2]) != np.sign(self.route[0][2] -
                                                                             self.target_lane_index[2]):
                return False
            # Unsafe braking required
            elif self_pred_a < -self.LANE_CHANGE_MAX_BRAKING_IMPOSED:
                return False

        # Is there an acceleration advantage for me and/or my followers to change lane?
        else:
            self_a = self.acceleration(ego_vehicle=self, front_vehicle=old_preceding)
            old_following_a = self.acceleration(ego_vehicle=old_following, front_vehicle=self)
            old_following_pred_a = self.acceleration(ego_vehicle=old_following, front_vehicle=old_preceding)
            jerk = self_pred_a - self_a + self.POLITENESS * (
                    new_following_pred_a - new_following_a + old_following_pred_a - old_following_a
            )
            if jerk < self.LANE_CHANGE_MIN_ACC_GAIN:
                return False

        # All clear, let's go!
        return True

    def recover_from_stop(self, acceleration: float) -> float:
        """
        If stopped on the wrong lane, try a reversing maneuver.

        :param acceleration: desired acceleration from IDM
        :return: suggested acceleration to recover from being stuck
        """
        stopped_speed = 5
        safe_distance = 200
        # Is the vehicle stopped on the wrong lane?
        if self.target_lane_index != self.lane_index and self.speed < stopped_speed:
            _, rear = self.traffic_mgr.neighbour_vehicles(self)
            _, new_rear = self.traffic_mgr.neighbour_vehicles(
                self, self.traffic_mgr.current_map.road_network.get_lane(self.target_lane_index)
            )
            # Check for free room behind on both lanes
            if (not rear or rear.lane_distance_to(self) > safe_distance) and \
                    (not new_rear or new_rear.lane_distance_to(self) > safe_distance):
                # Reverse
                return -self.COMFORT_ACC_MAX / 2
        return acceleration
