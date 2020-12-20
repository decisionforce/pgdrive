from collections import deque
from typing import Union, List

import numpy as np

import pgdrive.utils.math_utils as utils
from pgdrive.utils import get_np_random
from pgdrive.scene_creator.lanes.lane import AbstractLane
from pgdrive.scene_creator.road_object.object import Landmark, Obstacle, RoadObject
from pgdrive.scene_manager.traffic_manager import TrafficManager, LaneIndex


class Vehicle:
    """
    A moving vehicle on a road, and its kinematics.

    The vehicle is represented by a dynamical system: a modified bicycle model.
    It's state is propagated depending on its steering and acceleration actions.
    """

    COLLISIONS_ENABLED = True
    """ Enable collision detection between vehicles """

    LENGTH = 5.0
    """ Vehicle length [m] """
    WIDTH = 2.0
    """ Vehicle width [m] """
    DEFAULT_SPEEDS = [23, 25]
    """ Range for random initial speeds [m/s] """
    MAX_SPEED = 40.
    """ Maximum reachable speed [m/s] """
    def __init__(
        self,
        scene: TrafficManager,
        position: List,
        heading: float = 0,
        speed: float = 0,
        np_random: np.random.RandomState = None,
    ):
        self.scene = scene
        self._position = np.array(position).astype('float')
        self.heading = heading
        self.speed = speed
        self.lane_index = self.scene.network.get_closest_lane_index(self.position) if self.scene else np.nan
        self.lane = self.scene.network.get_lane(self.lane_index) if self.scene else None
        self.action = {'steering': 0, 'acceleration': 0}
        self.crashed = False
        self.log = []
        self.history = deque(maxlen=30)
        assert np_random is not None
        self.np_random = np_random if np_random else get_np_random()

    @property
    def position(self):
        return self._position.copy()

    def set_position(self, pos):
        self._position = np.asarray(pos).copy()

    @classmethod
    def make_on_lane(cls, scene: TrafficManager, lane_index: LaneIndex, longitudinal: float, speed: float = 0):
        """
        Create a vehicle on a given lane at a longitudinal position.

        :param scene: the road where the vehicle is driving
        :param lane_index: index of the lane where the vehicle is located
        :param longitudinal: longitudinal position along the lane
        :param speed: initial speed in [m/s]
        :return: A vehicle with at the specified position
        """
        lane = scene.network.get_lane(lane_index)
        if speed is None:
            speed = lane.speed_limit
        return cls(scene, lane.position(longitudinal, 0), lane.heading_at(longitudinal), speed)

    @classmethod
    def create_random(
        cls, scene: TrafficManager, lane: AbstractLane, longitude: float, speed: float = None, random_seed=None
    ):
        """
        Create a random vehicle on the road.

        The lane and /or speed are chosen randomly, while longitudinal position is chosen behind the last
        vehicle in the road with density based on the number of lanes.

        :param longitude: the longitude on lane
        :param lane: the lane where the vehicle is born
        :param scene: the scene where the vehicle is driving
        :param speed: initial speed in [m/s]. If None, will be chosen randomly
        :return: A vehicle with random position and/or speed
        """
        if speed is None:
            speed = scene.np_random.uniform(Vehicle.DEFAULT_SPEEDS[0], Vehicle.DEFAULT_SPEEDS[1])
        v = cls(
            scene,
            list(lane.position(longitude, 0)),
            lane.heading_at(longitude),
            speed,
            np_random=get_np_random(random_seed)
        )
        return v

    @classmethod
    def create_from(cls, vehicle: "Vehicle") -> "Vehicle":
        """
        Create a new vehicle from an existing one.

        Only the vehicle dynamics are copied, other properties are default.

        :param vehicle: a vehicle
        :return: a new vehicle at the same dynamical state
        """
        v = cls(vehicle.scene, vehicle.position, vehicle.heading, vehicle.speed)
        return v

    def act(self, action: Union[dict, str] = None) -> None:
        """
        Store an action to be repeated.

        :param action: the input action
        """
        if action:
            self.action = action

    def step(self, dt: float) -> None:
        """
        Propagate the vehicle state given its actions.

        Integrate a modified bicycle model with a 1st-order response on the steering wheel dynamics.
        If the vehicle is crashed, the actions are overridden with erratic steering and braking until complete stop.
        The vehicle's current lane is updated.

        :param dt: timestep of integration of the model [s]
        """
        self.clip_actions()
        delta_f = self.action['steering']
        beta = np.arctan(1 / 2 * np.tan(delta_f))
        v = self.speed * np.array([np.cos(self.heading + beta), np.sin(self.heading + beta)])
        self._position += v * dt
        self.heading += self.speed * np.sin(beta) / (self.LENGTH / 2) * dt
        self.speed += self.action['acceleration'] * dt
        # for performance reason,
        # self.on_state_update()

    def clip_actions(self) -> None:
        if self.crashed:
            self.action['steering'] = 0
            self.action['acceleration'] = -1.0 * self.speed
        self.action['steering'] = float(self.action['steering'])
        self.action['acceleration'] = float(self.action['acceleration'])
        if self.speed > self.MAX_SPEED:
            self.action['acceleration'] = min(self.action['acceleration'], 1.0 * (self.MAX_SPEED - self.speed))
        elif self.speed < -self.MAX_SPEED:
            self.action['acceleration'] = max(self.action['acceleration'], 1.0 * (self.MAX_SPEED - self.speed))

    def on_state_update(self) -> None:
        if self.scene:
            self.lane_index = self.scene.network.get_closest_lane_index(self.position)
            self.lane = self.scene.network.get_lane(self.lane_index)

    def lane_distance_to(self, vehicle: "Vehicle", lane: AbstractLane = None) -> float:
        """
        Compute the signed distance to another vehicle along a lane.

        :param vehicle: the other vehicle
        :param lane: a lane
        :return: the distance to the other vehicle [m]
        """
        if not vehicle:
            return np.nan
        if not lane:
            lane = self.lane
        return lane.local_coordinates(vehicle.position)[0] - lane.local_coordinates(self.position)[0]

    def check_collision(self, other: Union['Vehicle', 'RoadObject']) -> None:
        """
        Check for collision with another vehicle.

        :param other: the other vehicle or object
        """
        if self.crashed or other is self:
            return

        if isinstance(other, Vehicle):
            if not self.COLLISIONS_ENABLED or not other.COLLISIONS_ENABLED:
                return

            if self._is_colliding(other):
                self.speed = other.speed = min([self.speed, other.speed], key=abs)
                self.crashed = other.crashed = True
        elif isinstance(other, Obstacle):
            if not self.COLLISIONS_ENABLED:
                return

            if self._is_colliding(other):
                self.speed = min([self.speed, 0], key=abs)
                self.crashed = other.hit = True
        elif isinstance(other, Landmark):
            if self._is_colliding(other):
                other.hit = True

    def _is_colliding(self, other):
        # Fast spherical pre-check
        if np.linalg.norm(other.position - self.position) > self.LENGTH:
            return False
        # Accurate rectangular check
        return utils.rotated_rectangles_intersect(
            (self.position, 0.9 * self.LENGTH, 0.9 * self.WIDTH, self.heading),
            (other.position, 0.9 * other.LENGTH, 0.9 * other.WIDTH, other.heading)
        )

    @property
    def direction(self) -> np.ndarray:
        return np.array([np.cos(self.heading), np.sin(self.heading)])

    @property
    def velocity(self) -> np.ndarray:
        return self.speed * self.direction

    @property
    def destination(self) -> np.ndarray:
        if getattr(self, "route", None):
            last_lane = self.scene.network.get_lane(self.route[-1])
            return last_lane.position(last_lane.length, 0)
        else:
            return self.position

    @property
    def destination_direction(self) -> np.ndarray:
        if (self.destination != self.position).any():
            return (self.destination - self.position) / np.linalg.norm(self.destination - self.position)
        else:
            return np.zeros((2, ))

    @property
    def on_road(self) -> bool:
        """ Is the vehicle on its current lane, or off-scene ? """
        return self.lane.on_lane(self.position)

    def front_distance_to(self, other: "Vehicle") -> float:
        return self.direction.dot(other.position - self.position)

    def to_dict(self, origin_vehicle: "Vehicle" = None, observe_intentions: bool = True) -> dict:
        d = {
            'presence': 1,
            'x': self.position[0],
            'y': self.position[1],
            'vx': self.velocity[0],
            'vy': self.velocity[1],
            'cos_h': self.direction[0],
            'sin_h': self.direction[1],
            'cos_d': self.destination_direction[0],
            'sin_d': self.destination_direction[1]
        }
        if not observe_intentions:
            d["cos_d"] = d["sin_d"] = 0
        if origin_vehicle:
            origin_dict = origin_vehicle.to_dict()
            for key in ['x', 'y', 'vx', 'vy']:
                d[key] -= origin_dict[key]
        return d

    def __str__(self):
        return "{} #{}: {}".format(self.__class__.__name__, id(self) % 1000, self.position)

    def __repr__(self):
        return self.__str__()
