import numpy as np

from pgdrive.component.vehicle_module.PID_controller import PIDController
from pgdrive.policy.base_policy import BasePolicy
from pgdrive.utils.math_utils import not_zero, wrap_to_pi, point_distance
from pgdrive.utils.scene_utils import is_same_lane_index, is_following_lane_index


class IDMPolicy(BasePolicy):
    """
    We implement this policy based on the HighwayEnv code base.
    """
    TAU_ACC = 0.6  # [s]
    TAU_HEADING = 0.3  # [s]
    TAU_LATERAL = 0.8  # [s]

    TAU_PURSUIT = 0.5 * TAU_HEADING  # [s]
    KP_A = 1 / TAU_ACC
    KP_HEADING = 1 / TAU_HEADING
    KP_LATERAL = 1 / TAU_LATERAL  # [1/s]
    MAX_STEERING_ANGLE = np.pi / 3  # [rad]
    DELTA_SPEED = 5  # [m/s]

    COMFORT_ACC_MAX = 3.0  # [m/s2]
    """Desired maximum acceleration."""

    COMFORT_ACC_MIN = -5.0  # [m/s2]
    """Desired maximum deceleration."""

    DISTANCE_WANTED = 5.0
    """Desired jam distance to the front vehicle."""

    TIME_WANTED = 1.5  # [s]
    """Desired time gap to the front v"""

    DELTA = 2.0  # []
    """Exponent of the velocity term."""

    DELTA_RANGE = [3.5, 4.5]
    """Range of delta when chosen randomly."""

    # Lateral policy parameters
    LANE_CHANGE_FREQ = 15  # [s]
    LANE_CHANGE_SPEED_INCREASE = 5

    # Normal speed
    NORMAL_SPEED = 30

    # Creep Speed
    CREEP_SPEED = 10

    # Lane Change Speed
    LANE_CHANGE_SPEED = 20

    SAFE_LANE_CHANGE_DISTANCE = 5

    def __init__(self, control_object, random_seed):
        super(IDMPolicy, self).__init__(control_object=control_object, random_seed=random_seed)
        self.target_speed = self.NORMAL_SPEED
        self.steering_target_lane = None  #
        self.routing_target_lane = None
        self.available_routing_index_range = None
        self.lane_change_timer = self.np_random.randint(0, self.LANE_CHANGE_FREQ)

        self.heading_pid = PIDController(1.7, 0.01, 3.5)
        self.lateral_pid = PIDController(0.3, .002, 0.05)

    def act(self, *args, **kwargs):
        self.move_to_next_road()
        surrounding_objs = self.control_object.lidar.get_surrounding_objects(self.control_object)
        idx = self.routing_target_lane.index[-1]
        left_lane = self.control_object.navigation.current_ref_lanes[idx - 1] if idx > 0 else None
        right_lane = self.control_object.navigation.current_ref_lanes[idx + 1] if idx + 1 < len(
            self.control_object.navigation.current_ref_lanes) else None
        front_objs, dist = self.find_front_objs(surrounding_objs, [left_lane, self.routing_target_lane, right_lane],
                                                self.control_object.position)
        self.lane_change_policy(surrounding_objs, front_objs, dist)
        steering = self.steering_control()
        acc = self.acceleration(self.control_object, front_objs[1], dist[1])
        return [steering, acc]

    def move_to_next_road(self):
        current_lanes = self.control_object.navigation.current_ref_lanes
        if self.routing_target_lane is None:
            self.routing_target_lane = self.control_object.lane
        if self.control_object.lane is not self.routing_target_lane:
            for lane in current_lanes:
                if self.routing_target_lane.is_previous_lane_of(
                        lane):
                    # two lanes connect
                    self.routing_target_lane = lane
                    self.steering_target_lane = lane
                    return
                    # lane change for lane num change

    def steering_control(self) -> float:
        # heading control following a lateral distance control
        ego_vehicle = self.control_object
        target_lane = self.steering_target_lane
        long, lat = target_lane.local_coordinates(ego_vehicle.position)
        lane_heading = target_lane.heading_at(long + 1)
        v_heading = ego_vehicle.heading_theta
        steering = self.heading_pid.get_result(wrap_to_pi(lane_heading - v_heading))
        steering += self.lateral_pid.get_result(-lat)
        return float(steering)

    def acceleration(self, ego_vehicle, front_obj, dist_to_front) -> float:
        ego_target_speed = not_zero(self.target_speed, 0)
        acceleration = self.COMFORT_ACC_MAX * (1 - np.power(max(ego_vehicle.speed, 0) / ego_target_speed, self.DELTA))
        if front_obj:
            d = dist_to_front
            speed_diff = self.desired_gap(ego_vehicle, front_obj) / not_zero(d)
            acceleration -= self.COMFORT_ACC_MAX * (speed_diff ** 2)
        return acceleration

    def desired_gap(self, ego_vehicle, front_obj, projected: bool = True) -> float:
        d0 = self.DISTANCE_WANTED
        tau = self.TIME_WANTED
        ab = -self.COMFORT_ACC_MAX * self.COMFORT_ACC_MIN
        dv = np.dot(ego_vehicle.velocity - front_obj.velocity, ego_vehicle.heading) if projected \
            else ego_vehicle.speed - front_obj.speed
        d_star = d0 + ego_vehicle.speed * tau + ego_vehicle.speed * dv / (2 * np.sqrt(ab))
        return d_star

    @staticmethod
    def find_front_objs(objs, lanes, position):
        """
        Find objects in front of lanes, return objs, dist
        """
        min_long = [1000 if lane is not None else None for lane in lanes]
        ret = [None, None, None]
        find_in_current_lane = [False, False, False]
        current_long = [lane.local_coordinates(position)[0] if lane is not None else None for lane in lanes]
        left_long = [lane.length - current_long[idx] if lane is not None else None for idx, lane in enumerate(lanes)]

        for i, lane in enumerate(lanes):
            if lane is None:
                continue
            for obj in objs:
                if obj.lane is lane:
                    long = lane.local_coordinates(obj.position)[0] - current_long[i]
                    if min_long[i] > long > 0:
                        min_long[i] = long
                        ret[i] = obj
                        find_in_current_lane[i] = True
                elif not find_in_current_lane[i] and lane.is_previous_lane_of(obj.lane):
                    long = obj.lane.local_coordinates(obj.position)[0] + left_long[i]
                    if min_long[i] > long > 0:
                        min_long[i] = long
                        ret[i] = obj
        return ret, min_long

    def reset(self):
        self.heading_pid.reset()
        self.lateral_pid.reset()
        self.target_speed = self.NORMAL_SPEED
        self.steering_target_lane = None  #
        self.routing_target_lane = None
        self.available_routing_index_range = None
        self.lane_change_timer = self.np_random.randint(0, self.LANE_CHANGE_FREQ)

    def lane_change_policy(self, surrounding_objs, front_objs, dist):
        current_lanes = self.control_object.navigation.current_ref_lanes
        self.available_routing_index_range = [i for i in range(len(current_lanes))]
        next_lanes = self.control_object.navigation.next_ref_lanes
        lane_num_diff = len(current_lanes) - len(next_lanes) if next_lanes is not None else 0
        if lane_num_diff > 0:
            # lane num decrease, lane change may be needed
            if current_lanes[0].is_previous_lane_of(next_lanes[0]):
                index_range = [i for i in range(len(next_lanes))]
            else:
                index_range = [i for i in range(lane_num_diff, len(current_lanes))]
            self.available_routing_index_range = index_range
            if self.routing_target_lane.index[-1] not in index_range:
                # change lane
                if self.routing_target_lane.index[-1] > index_range[-1]:
                    self.routing_target_lane = current_lanes[self.routing_target_lane.index[-1] - 1]
                else:
                    self.routing_target_lane = current_lanes[self.routing_target_lane.index[-1] + 1]

        if self.routing_target_lane is not self.control_object.lane:
            # perform lane change at a proper time
            objs_on_routing_lane = self.get_objects_on_lane(surrounding_objs, lane=self.routing_target_lane,
                                                            distance=self.SAFE_LANE_CHANGE_DISTANCE)
            if len(objs_on_routing_lane) == 0:
                # perform lane change
                self.steering_target_lane = self.routing_target_lane
                self.target_speed = self.LANE_CHANGE_SPEED
            else:
                # creep to wait
                self.target_speed = self.CREEP_SPEED
                self.steering_target_lane = self.control_object.lane
        else:
            # lane follow or active change lane for high driving speed
            if abs(self.control_object.speed - self.NORMAL_SPEED) < 1 or front_objs[1] is None or abs(
                    front_objs[1].speed - self.NORMAL_SPEED) < 1 or (
                    dist[0] is not None and dist[0] < self.SAFE_LANE_CHANGE_DISTANCE and dist[-1] is not None and dist[
                -1] < self.SAFE_LANE_CHANGE_DISTANCE) or self.lane_change_timer < self.LANE_CHANGE_FREQ:
                # already max speed lane follow
                self.steering_target_lane = self.control_object.lane
                self.target_speed = self.NORMAL_SPEED
                self.lane_change_timer += 1
            else:
                # may lane change
                right_front_speed = front_objs[-1].speed if front_objs[-1] is not None else 1000 \
                    if dist[-1] is not None else None
                front_speed = front_objs[1].speed if front_objs[1] is not None else 1000 \
                    if dist[1] is not None else None
                left_front_speed = front_objs[0].speed if front_objs[0] is not None else 1000 \
                    if dist[0] is not None else None
                if left_front_speed is not None and left_front_speed - front_speed > self.LANE_CHANGE_SPEED_INCREASE:
                    # left overtake has a high priority
                    expect_lane_idx = current_lanes.index(self.control_object.lane) - 1
                    if expect_lane_idx in self.available_routing_index_range:
                        self.steering_target_lane = current_lanes[expect_lane_idx]
                        self.routing_target_lane = self.steering_target_lane
                        self.lane_change_timer = 0
                elif right_front_speed is not None and right_front_speed - front_speed > self.LANE_CHANGE_SPEED_INCREASE:
                    expect_lane_idx = current_lanes.index(self.control_object.lane) + 1
                    if expect_lane_idx in self.available_routing_index_range:
                        self.steering_target_lane = current_lanes[expect_lane_idx]
                        self.routing_target_lane = self.steering_target_lane
                        self.lane_change_timer = 0

                else:
                    # keep current lane
                    self.steering_target_lane = self.control_object.lane
                    self.target_speed = self.NORMAL_SPEED
                    self.lane_change_timer += 1

    def get_objects_on_lane(self, surrounding_objs, lane, distance=10):
        objs_on_routing_lane = []
        for obj in surrounding_objs:
            if (obj.lane is lane or obj.lane.is_previous_lane_of(lane) or lane.is_previous_lane_of(
                    obj.lane)) and point_distance(obj.position,
                                                  self.control_object.position) < distance:
                objs_on_routing_lane.append(obj)
        return objs_on_routing_lane
