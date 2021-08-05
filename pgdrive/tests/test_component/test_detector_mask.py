import copy
import logging
import math
from collections import defaultdict
from collections import namedtuple

from pgdrive.utils import import_cutils

cutils = import_cutils()

detect_result = namedtuple("detect_result", "cloud_points detected_objects")

import numpy as np

from pgdrive.component.vehicle.base_vehicle import BaseVehicle
from pgdrive.envs import PGDriveEnvV2

from pgdrive.utils import panda_position


class DetectorMask:
    def __init__(self, num_lasers: int, max_span: float, max_distance: float = 1e6):
        logging.warning(
            "This class is deprecated, the new lidar is implemented with this optimization, and"
            "can be easily extended to 3d"
        )
        self.num_lasers = num_lasers
        self.angle_delta = 360 / self.num_lasers
        # self.max_span = max_span
        self.half_max_span_square = (max_span / 2)**2
        self.masks = defaultdict(lambda: np.zeros((self.num_lasers, ), dtype=np.bool))
        # self.max_distance = max_distance + max_span
        self.max_distance_square = (max_distance + max_span)**2

    def update_mask(self, position_dict: dict, heading_dict: dict, is_target_vehicle_dict: dict):
        assert set(position_dict.keys()) == set(heading_dict.keys()) == set(is_target_vehicle_dict.keys())
        if not position_dict:
            return

        for k in self.masks.keys():
            self.masks[k].fill(False)

        for k, is_target in is_target_vehicle_dict.items():
            if is_target:
                self.masks[k]  # Touch to create entry

        keys = list(position_dict.keys())
        for c1, k1 in enumerate(keys[:-1]):
            for c2, k2 in enumerate(keys[c1 + 1:]):

                if (not is_target_vehicle_dict[k1]) and (not is_target_vehicle_dict[k2]):
                    continue

                pos1 = position_dict[k1]
                pos2 = position_dict[k2]
                head1 = heading_dict[k1]
                head2 = heading_dict[k2]

                diff = (pos2[0] - pos1[0], pos2[1] - pos1[1])
                dist_square = diff[0]**2 + diff[1]**2
                if dist_square < self.half_max_span_square:
                    if is_target_vehicle_dict[k1]:
                        self._mark_all(k1)
                    if is_target_vehicle_dict[k2]:
                        self._mark_all(k2)
                    continue

                if dist_square > self.max_distance_square:
                    continue

                span = None
                if is_target_vehicle_dict[k1]:
                    span = math.asin(math.sqrt(self.half_max_span_square / dist_square))
                    # relative heading of v2's center when compared to v1's center
                    relative_head = math.atan2(diff[1], diff[0])
                    head_in_1 = relative_head - head1
                    head_in_1_max = head_in_1 + span
                    head_in_1_min = head_in_1 - span
                    head_1_max = np.rad2deg(head_in_1_max)
                    head_1_min = np.rad2deg(head_in_1_min)
                    self._mark_this_range(head_1_min, head_1_max, name=k1)

                if is_target_vehicle_dict[k2]:
                    if span is None:
                        span = math.asin(math.sqrt(self.half_max_span_square / dist_square))
                    diff2 = (-diff[0], -diff[1])
                    # relative heading of v2's center when compared to v1's center
                    relative_head2 = math.atan2(diff2[1], diff2[0])
                    head_in_2 = relative_head2 - head2
                    head_in_2_max = head_in_2 + span
                    head_in_2_min = head_in_2 - span
                    head_2_max = np.rad2deg(head_in_2_max)
                    head_2_min = np.rad2deg(head_in_2_min)
                    self._mark_this_range(head_2_min, head_2_max, name=k2)

    def _mark_this_range(self, small_angle, large_angle, name):
        # We use clockwise to determine small and large angle.
        # For example, if you wish to fill 355 deg to 5 deg, then small_angle is 355, large_angle is 5.
        small_angle = small_angle % 360
        large_angle = large_angle % 360

        assert 0 <= small_angle <= 360
        assert 0 <= large_angle <= 360

        small_index = math.floor(small_angle / self.angle_delta)
        large_index = math.ceil(large_angle / self.angle_delta)
        if large_angle < small_angle:  # We are in the case like small=355, large=5
            self.masks[name][small_index:] = True
            self.masks[name][:large_index + 1] = True
        else:
            self.masks[name][small_index:large_index + 1] = True

    def _mark_all(self, name):
        self.masks[name].fill(True)

    def get_mask(self, name):
        assert name in self.masks, "It seems that you have not initialized the mask for vehicle {} yet!".format(name)
        return self.masks[name]

    def clear(self):
        self.masks.clear()

    def get_mask_ratio(self):
        total = 0
        masked = 0
        for k, v in self.masks.items():
            total += v.size
            masked += v.sum()
        return masked / total


def _line_intersect(theta, center, point1, point2, maximum: float = 10000):
    """
    theta: the direction of the laser
    center: the center of the source of laser
    point1: one point of the line intersection
    point2: another poing of the line intersection
    maximum: the return value if no intersection
    """
    x0, y0 = center
    x1, y1 = point1
    x2, y2 = point2
    dx1 = np.cos(theta)
    dy1 = np.sin(theta)
    dx2 = x2 - x1
    dy2 = y2 - y1
    DET = -dx1 * dy2 + dy1 * dx2
    if abs(DET) < 1e-9:
        return maximum
    r = 1.0 / DET * (-dy2 * (x1 - x0) + dx2 * (y1 - y0))
    s = 1.0 / DET * (-dy1 * (x1 - x0) + dx1 * (y1 - y0))
    if 0 - 1e-5 < s < 1 + 1e-5 and r >= 0:
        return r
    return maximum


def _search_angle(point1, point2, num_lasers, start, heading, perceive_distance: float = 50):
    laser_heading = np.arange(0, num_lasers) * 2 * np.pi / num_lasers + heading
    result = []
    for laser_index in range(num_lasers):
        ret = _line_intersect(theta=laser_heading[laser_index], center=start, point1=point1, point2=point2, maximum=1e8)
        assert 0 <= ret < 1e9
        if ret <= 1e8:
            result.append(ret / 1e8)
        else:
            result.append(1.0)
    return np.asarray(result)


def _test_mask(mask, stick_1_heading_deg, stick_2_heading_deg, max_span, stick1_x, stick2_x):
    stick_1_heading = np.deg2rad(stick_1_heading_deg)
    stick_2_heading = np.deg2rad(stick_2_heading_deg)
    stick_1_heading = stick_1_heading % (2 * np.pi)
    stick_2_heading = stick_2_heading % (2 * np.pi)

    stick1_pos = (stick1_x, 0)
    stick2_pos = (stick2_x, 0)
    mask.update_mask(
        position_dict={
            "stick1": stick1_pos,
            "stick2": stick2_pos
        },
        heading_dict={
            "stick1": stick_1_heading,
            "stick2": stick_2_heading
        },
        is_target_vehicle_dict={
            "stick1": True,
            "stick2": True
        }
    )
    mask_1 = mask.get_mask("stick1")
    mask_2 = mask.get_mask("stick2")

    left_of_stick2 = (stick2_pos[0], -max_span / 2)
    right_of_stick2 = (stick2_pos[0], max_span / 2)

    real_mask_1 = _search_angle(
        point1=left_of_stick2,
        point2=right_of_stick2,
        num_lasers=360,
        start=stick1_pos,
        heading=stick_1_heading,
        perceive_distance=100 * max_span
    ) < 1.0
    res = np.stack([real_mask_1, mask_1])
    assert all(mask_1[real_mask_1])  # mask 1 should at least include all True of real mask.
    if abs(stick1_x - stick2_x) > max_span:
        assert sum(abs(mask_1.astype(int) - real_mask_1.astype(int))) <= 3

    left_of_stick1 = (stick1_pos[0], -max_span / 2)
    right_of_stick1 = (stick1_pos[0], max_span / 2)

    real_mask_2 = _search_angle(
        point1=left_of_stick1,
        point2=right_of_stick1,
        num_lasers=360,
        start=stick2_pos,
        heading=stick_2_heading,
        perceive_distance=100 * max_span
    ) < 1.0
    res2 = np.stack([real_mask_2, mask_2])
    assert all(mask_2[real_mask_2])  # mask 1 should at least include all True of real mask.
    if abs(stick1_x - stick2_x) > max_span:
        assert sum(abs(mask_2.astype(int) - real_mask_2.astype(int))) <= 3


def test_detector_mask():
    # A infinite long (1e7) stick 2 in front of (0.01m) stick 1.

    pos_xy = [0, -1, 1, -100, 100]
    angles = [0, 0.01, 30, 89, 90, 91, 130, 180, 181, 270, 360, 400]
    angles += [-a for a in angles]

    mask = DetectorMask(num_lasers=360, max_span=1e7)
    _test_mask(mask, -270, 30, 1e7, 0, -1)

    mask = DetectorMask(num_lasers=360, max_span=1)
    _test_mask(mask, -180, -300, 1, 0, -1)
    _test_mask(mask, -361, 270, 1, 0, -100)

    for max_span in [1e7, 1, 0.1]:
        mask = DetectorMask(num_lasers=360, max_span=max_span)
        for pos1_x in pos_xy:
            for pos2_x in pos_xy:
                angles1 = np.random.choice(angles, 5)
                angles2 = np.random.choice(angles, 5)
                for stick_1_heading_deg in angles1:
                    for stick_2_heading_deg in angles2:
                        _test_mask(mask, stick_1_heading_deg, stick_2_heading_deg, max_span, pos1_x, pos2_x)
                print("Finish. ", max_span, pos1_x, pos2_x)


def test_detector_mask_in_lidar():
    env = PGDriveEnvV2({"traffic_density": 1.0, "map": "SSSSS", "random_traffic": False})
    try:
        env.reset()
        span = 2 * max(env.vehicle.WIDTH, env.vehicle.LENGTH)
        detector_mask = DetectorMask(
            env.vehicle.config.lidar.num_lasers, span, max_distance=env.vehicle.config.lidar.distance
        )
        ep_count = 0
        for _ in range(3000):
            o, r, d, i = env.step([0, 1])

            print("We have: {} vehicles!".format(env.engine.traffic_manager.get_vehicle_num()))

            v = env.vehicle
            c_p, objs = v.lidar.perceive(v, detector_mask=None)
            old_cloud_points = np.array(copy.deepcopy(c_p))

            position_dict = {}
            heading_dict = {}
            is_target_vehicle_dict = {}
            for v in env.engine.traffic_manager.vehicles:
                position_dict[v.name] = v.position
                heading_dict[v.name] = v.heading_theta
                is_target_vehicle_dict[v.name] = True if isinstance(v, BaseVehicle) else False

            detector_mask.update_mask(
                position_dict=position_dict, heading_dict=heading_dict, is_target_vehicle_dict=is_target_vehicle_dict
            )

            real_mask = old_cloud_points != 1.0
            mask = detector_mask.get_mask(env.vehicle.name)
            stack = np.stack([old_cloud_points, real_mask, mask])
            if not all(mask[real_mask]):
                print('stop')
            assert all(mask[real_mask])  # mask 1 should at least include all True of real mask.

            print(
                "Num of true in our mask: {}, in old mask: {}. Overlap: {}. We have {} more.".format(
                    sum(mask.astype(int)), sum(real_mask.astype(int)), sum(mask[real_mask].astype(int)),
                    sum(mask.astype(int)) - sum(real_mask.astype(int))
                )
            )

            # assert sum(abs(mask.astype(int) - real_mask.astype(int))) <= 3
            v = env.vehicle
            c_p, objs = v.lidar.perceive(v)
            new_cloud_points = np.array(copy.deepcopy(c_p))
            np.testing.assert_almost_equal(old_cloud_points, new_cloud_points)

            if d:
                env.reset()
                ep_count += 1
                if ep_count == 3:
                    break
    finally:
        env.close()


def test_cutils_lidar():
    def _old_perceive(
        self,
        vehicle_position,
        heading_theta,
        physics_world,
        extra_filter_node: set = None,
        detector_mask: np.ndarray = None
    ):
        """
        Call me to update the perception info
        """
        assert detector_mask is not "WRONG"
        # coordinates problem here! take care
        extra_filter_node = extra_filter_node or set()
        pg_start_position = panda_position(vehicle_position, self.height)

        # init
        self.cloud_points.fill(1.0)
        self.detected_objects = []

        # lidar calculation use pg coordinates
        mask = self.mask
        # laser_heading = self._lidar_range + heading_theta
        # point_x = self.perceive_distance * np.cos(laser_heading) + vehicle_position[0]
        # point_y = self.perceive_distance * np.sin(laser_heading) + vehicle_position[1]

        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        for laser_index in range(self.num_lasers):
            # # coordinates problem here! take care

            if (detector_mask is not None) and (not detector_mask[laser_index]):
                # update vis
                if self.cloud_points_vis is not None:
                    laser_end = self._get_laser_end(laser_index, heading_theta, vehicle_position)
                    self._add_cloud_point_vis(laser_index, laser_end)
                continue

            laser_end = self._get_laser_end(laser_index, heading_theta, vehicle_position)
            result = physics_world.rayTestClosest(pg_start_position, laser_end, mask)
            node = result.getNode()
            if node in extra_filter_node:
                # Fall back to all tests.
                results = physics_world.rayTestAll(pg_start_position, laser_end, mask)
                hits = results.getHits()
                hits = sorted(hits, key=lambda ret: ret.getHitFraction())
                for result in hits:
                    if result.getNode() in extra_filter_node:
                        continue
                    self.detected_objects.append(result)
                    self.cloud_points[laser_index] = result.getHitFraction()
                    break
            else:
                hits = result.hasHit()
                self.cloud_points[laser_index] = result.getHitFraction()
                if node:
                    self.detected_objects.append(result)

            # update vis
            if self.cloud_points_vis is not None:
                self._add_cloud_point_vis(laser_index, result.getHitPos() if hits else laser_end)
        return self.cloud_points

    from pgdrive.utils.cutils import _get_fake_cutils
    _fake_cutils = _get_fake_cutils()

    def fake_cutils_perceive(
        self,
        vehicle_position,
        heading_theta,
        physics_world,
        extra_filter_node: set = None,
        detector_mask: np.ndarray = None
    ):
        cloud_points, _, _ = _fake_cutils.cutils_perceive(
            cloud_points=self.cloud_points,
            detector_mask=detector_mask.astype(dtype=np.uint8) if detector_mask is not None else None,
            mask=self.mask,
            lidar_range=self._lidar_range,
            perceive_distance=self.perceive_distance,
            heading_theta=heading_theta,
            vehicle_position_x=vehicle_position[0],
            vehicle_position_y=vehicle_position[1],
            num_lasers=self.num_lasers,
            height=self.height,
            physics_world=physics_world,
            extra_filter_node=extra_filter_node if extra_filter_node else set(),
            require_colors=self.cloud_points_vis is not None,
            ANGLE_FACTOR=self.ANGLE_FACTOR,
            MARK_COLOR0=self.MARK_COLOR[0],
            MARK_COLOR1=self.MARK_COLOR[1],
            MARK_COLOR2=self.MARK_COLOR[2]
        )
        return cloud_points.tolist(), _

    env = PGDriveEnvV2({"map": "C", "traffic_density": 1.0, "environment_num": 10, "use_render": False})
    env.reset()
    env.vehicle.lidar.cloud_points = np.ones((env.vehicle.lidar.num_lasers, ), dtype=float)
    env.vehicle.lidar.detected_objects = []
    try:
        for _ in range(3):
            env.reset()
            ep_count = 0
            for _ in range(3000):
                o, r, d, i = env.step([0, 1])

                v = env.vehicle
                new_cloud_points, _ = v.lidar.perceive(v, detector_mask=None)
                new_cloud_points = np.array(copy.deepcopy(new_cloud_points))
                old_cloud_points = _old_perceive(
                    v.lidar, v.position, v.heading_theta, v.engine.physics_world.dynamic_world, {v.chassis.node()}, None
                )
                np.testing.assert_almost_equal(new_cloud_points, old_cloud_points)

                fake_cutils_cloud_points, _ = fake_cutils_perceive(
                    v.lidar,
                    v.position,
                    v.heading_theta,
                    v.engine.physics_world.dynamic_world,
                    extra_filter_node={v.chassis.node()},
                    detector_mask=None
                )
                np.testing.assert_almost_equal(new_cloud_points, fake_cutils_cloud_points)

                # assert sum(abs(mask.astype(int) - real_mask.astype(int))) <= 3
                v = env.vehicle
                c_p, _ = v.lidar.perceive(v)
                old_cloud_points = _old_perceive(
                    v.lidar, v.position, v.heading_theta, v.engine.physics_world.dynamic_world, {v.chassis.node()},
                    v.lidar._get_lidar_mask(v)
                )
                new_cloud_points = np.array(copy.deepcopy(c_p))
                np.testing.assert_almost_equal(old_cloud_points, new_cloud_points)

                if d:
                    env.reset()
                    ep_count += 1
                    if ep_count == 3:
                        break
    finally:
        env.close()


if __name__ == '__main__':
    # test_detector_mask()
    # test_detector_mask_in_lidar()
    test_cutils_lidar()
