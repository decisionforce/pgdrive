import math
from pgdrive.component.lane.abs_lane import AbstractLane
from typing import Set

import numpy as np
from panda3d.bullet import BulletGhostNode, ZUp, BulletCylinderShape
from panda3d.core import NodePath

from pgdrive.component.vehicle_module.distance_detector import DistanceDetector
from pgdrive.constants import BodyName, CamMask, CollisionGroup
from pgdrive.engine import get_engine
from pgdrive.utils.coordinates_shift import panda_position
from pgdrive.utils.utils import get_object_from_node


class Lidar(DistanceDetector):
    ANGLE_FACTOR = True
    Lidar_point_cloud_obs_dim = 240
    DEFAULT_HEIGHT = 0.5

    BROAD_PHASE_EXTRA_DIST = 0

    def __init__(self, num_lasers: int = 240, distance: float = 50, enable_show=False):
        super(Lidar, self).__init__(num_lasers, distance, enable_show)
        self.origin.hide(CamMask.RgbCam | CamMask.Shadow | CamMask.Shadow | CamMask.DepthCam)
        self.mask = CollisionGroup.TrafficVehicle | CollisionGroup.EgoVehicle | CollisionGroup.InvisibleWall

        # lidar can calculate the detector mask by itself
        self.angle_delta = 360 / num_lasers
        self.broad_detector = NodePath(BulletGhostNode("detector_mask"))
        self.broad_detector.node().addShape(BulletCylinderShape(self.BROAD_PHASE_EXTRA_DIST + distance, 5))
        self.broad_detector.node().setIntoCollideMask(CollisionGroup.LidarBroadDetector)
        self.broad_detector.node().setStatic(True)
        engine = get_engine()
        engine.physics_world.static_world.attach(self.broad_detector.node())
        self.enable_mask = True if not engine.global_config["_disable_detector_mask"] else False

    def perceive(self, base_vehicle, detector_mask=True):
        lidar_mask = self._get_lidar_mask(base_vehicle)[0] if detector_mask and self.enable_mask else None
        return super(Lidar, self).perceive(base_vehicle, base_vehicle.engine.physics_world.dynamic_world, lidar_mask)

    @staticmethod
    def get_surrounding_vehicles(detected_objects) -> Set:
        # TODO this will be removed in the future
        vehicles = set()
        objs = detected_objects
        for ret in objs:
            if ret.getNode().hasPythonTag(BodyName.Traffic_vehicle):
                vehicles.add(get_object_from_node(ret.getNode()).kinematic_model)
            elif ret.getNode().hasPythonTag(BodyName.Base_vehicle):
                vehicles.add(get_object_from_node(ret.getNode()))
        return vehicles

    def get_surrounding_vehicles_info(self, ego_vehicle, detected_objects, num_others: int = 4):
        from pgdrive.utils.math_utils import norm, clip
        surrounding_vehicles = list(self.get_surrounding_vehicles(detected_objects))
        surrounding_vehicles.sort(
            key=lambda v: norm(ego_vehicle.position[0] - v.position[0], ego_vehicle.position[1] - v.position[1])
        )
        surrounding_vehicles += [None] * num_others
        res = []
        for vehicle in surrounding_vehicles[:num_others]:
            if vehicle is not None:
                # assert isinstance(vehicle, IDMVehicle or Base), "Now PGDrive Doesn't support other vehicle type"
                relative_position = ego_vehicle.projection(vehicle.position - ego_vehicle.position)
                # It is possible that the centroid of other vehicle is too far away from ego but lidar shed on it.
                # So the distance may greater than perceive distance.
                res.append(clip((relative_position[0] / self.perceive_distance + 1) / 2, 0.0, 1.0))
                res.append(clip((relative_position[1] / self.perceive_distance + 1) / 2, 0.0, 1.0))

                relative_velocity = ego_vehicle.projection(vehicle.velocity - ego_vehicle.velocity)
                res.append(clip((relative_velocity[0] / ego_vehicle.max_speed + 1) / 2, 0.0, 1.0))
                res.append(clip((relative_velocity[1] / ego_vehicle.max_speed + 1) / 2, 0.0, 1.0))
            else:
                res += [0.0] * 4
        return res

    def _get_lidar_mask(self, vehicle):
        pos1 = vehicle.position
        head1 = vehicle.heading_theta

        mask = np.zeros((self.num_lasers, ), dtype=np.bool)
        mask.fill(False)
        objs = self.get_surrounding_objects(vehicle)
        for obj in objs:
            pos2 = obj.position
            length = obj.LENGTH if hasattr(obj, "LENGTH") else vehicle.LENGTH
            width = obj.WIDTH if hasattr(obj, "WIDTH") else vehicle.WIDTH
            half_max_span_square = ((length + width) / 2)**2
            diff = (pos2[0] - pos1[0], pos2[1] - pos1[1])
            dist_square = diff[0]**2 + diff[1]**2
            if dist_square < half_max_span_square:
                mask.fill(True)
                continue

            span = math.asin(math.sqrt(half_max_span_square / dist_square))
            # relative heading of v2's center when compared to v1's center
            relative_head = math.atan2(diff[1], diff[0])
            head_in_1 = relative_head - head1
            head_in_1_max = head_in_1 + span
            head_in_1_min = head_in_1 - span
            head_1_max = np.rad2deg(head_in_1_max)
            head_1_min = np.rad2deg(head_in_1_min)
            mask = self._mark_this_range(head_1_min, head_1_max, mask)

        return mask, objs

    def get_surrounding_objects(self, vehicle):
        self.broad_detector.setPos(panda_position(vehicle.position))
        physics_world = vehicle.engine.physics_world.dynamic_world
        contact_results = physics_world.contactTest(self.broad_detector.node(), True).getContacts()
        objs = set()
        for contact in contact_results:
            node0 = contact.getNode0()
            node1 = contact.getNode1()
            nodes = [node0, node1]
            nodes.remove(self.broad_detector.node())
            obj = get_object_from_node(nodes[0])
            if not isinstance(obj, AbstractLane) and obj is not None:
                objs.add(obj)
        if vehicle in objs:
            objs.remove(vehicle)
        return objs

    def _mark_this_range(self, small_angle, large_angle, mask):
        # We use clockwise to determine small and large angle.
        # For example, if you wish to fill 355 deg to 5 deg, then small_angle is 355, large_angle is 5.
        small_angle = small_angle % 360
        large_angle = large_angle % 360

        assert 0 <= small_angle <= 360
        assert 0 <= large_angle <= 360

        small_index = math.floor(small_angle / self.angle_delta)
        large_index = math.ceil(large_angle / self.angle_delta)
        if large_angle < small_angle:  # We are in the case like small=355, large=5
            mask[small_index:] = True
            mask[:large_index + 1] = True
        else:
            mask[small_index:large_index + 1] = True
        return mask

    def destroy(self):
        get_engine().physics_world.static_world.remove(self.broad_detector.node())
        self.broad_detector.removeNode()
        super(Lidar, self).destroy()
