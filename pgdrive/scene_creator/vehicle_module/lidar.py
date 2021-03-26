import logging
from typing import Set

import numpy as np
from panda3d.bullet import BulletGhostNode, BulletSphereShape, BulletAllHitsRayResult, BulletRayHit
from panda3d.core import BitMask32, NodePath
from pgdrive.constants import BodyName, CamMask, CollisionGroup
from pgdrive.scene_creator.highway_vehicle.behavior import IDMVehicle
from pgdrive.scene_creator.vehicle.traffic_vehicle import PGTrafficVehicle
from pgdrive.utils.asset_loader import AssetLoader
from pgdrive.utils.coordinates_shift import panda_position
from pgdrive.world.pg_physics_world import PGPhysicsWorld


class Lidar:
    Lidar_point_cloud_obs_dim = 240
    DEFAULT_HEIGHT = 0.2

    def __init__(self, parent_node_np: NodePath, num_lasers: int = 240, distance: float = 50, enable_show=False):
        show = enable_show and (AssetLoader.loader is not None)
        self.Lidar_point_cloud_obs_dim = num_lasers
        self.num_lasers = num_lasers

        # lidar properties
        self.perceive_distance = distance
        self.radian_unit = 2 * np.pi / num_lasers
        self.height = self.DEFAULT_HEIGHT

        # used to save detection results of each frame
        self.dynamic_elements = []
        self.dynamic_cloud_points = []
        self.static_elements = []
        self.static_points = []

        # visable
        self.node_path = parent_node_np.attachNewNode("cloudPoints")
        self.node_path.hide(CamMask.RgbCam | CamMask.Shadow | CamMask.Shadow | CamMask.DepthCam)
        self.cloud_points = [] if show else None
        logging.debug("Load Vehicle Module: {}".format(self.__class__.__name__))
        if show:
            for laser_debug in range(self.num_lasers):
                ball = AssetLoader.loader.loadModel(AssetLoader.file_path("models", "box.bam"))
                ball.setScale(0.001)
                ball.setColor(0., 0.5, 0.5, 1)
                shape = BulletSphereShape(0.1)
                ghost = BulletGhostNode('Lidar Point')
                ghost.setIntoCollideMask(BitMask32.allOff())
                ghost.addShape(shape)
                laser_np = self.node_path.attachNewNode(ghost)
                self.cloud_points.append(laser_np)
                ball.getChildren().reparentTo(laser_np)
            # self.node_path.flattenStrong()

    def perceive(self, vehicle_position, heading_theta, pg_physics_world: PGPhysicsWorld):
        """
        Call me to update the perception info
        """
        # coordinates problem here! take care
        pg_start_position = panda_position(vehicle_position, self.height)

        # lidar calculation use pg coordinates
        mask = BitMask32.bit(PGTrafficVehicle.COLLISION_MASK) | BitMask32.bit(CollisionGroup.EgoVehicle) | \
               BitMask32.bit(CollisionGroup.LaneLine)
        laser_heading = np.arange(0, self.num_lasers) * self.radian_unit + heading_theta
        point_x = self.perceive_distance * np.cos(laser_heading) + vehicle_position[0]
        point_y = self.perceive_distance * np.sin(laser_heading) + vehicle_position[1]
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        for laser_index in range(self.num_lasers):
            # # coordinates problem here! take care
            laser_end = panda_position((point_x[laser_index], point_y[laser_index]), self.height)
            results = pg_physics_world.dynamic_world.rayTestAll(pg_start_position, laser_end, mask)

            dynamic_point_pos = laser_end
            static_point_pos = laser_end
            for result  in results.getHits():
                if result.getNode().getName() ==BodyName.Sidewalk:
                    self.static_results.append(result)
                    static_point_pos = result.getHitPos()
                else:
                    self.dynamic_results.append(result)
                    dynamic_point_pos = result.getHitPos()



            if self.cloud_points is not None:
                # if 0<=laser_index < 10 or 230 <= laser_index <=239:
                #     self.cloud_points[laser_index].setColor(1,0,0)
                self.cloud_points[laser_index].setPos(curpos)
                f = laser_index / self.num_lasers
                self.cloud_points[laser_index].setColor(f*51/255, f*221/255, f)

    def get_surrounding_vehicles(self) -> Set[IDMVehicle]:
        vehicles = set()
        for ret in self.dynamic_results:
            if ret.hasHit() and ret.getNode().hasPythonTag(BodyName.Traffic_vehicle):
                vehicles.add(ret.getNode().getPythonTag(BodyName.Traffic_vehicle).kinematic_model)
        return vehicles

    # def _get_surrounding_objects(self) -> Set[Object]:
    #     """
    #     TODO may be static objects info should be added in obs, now this func is useless
    #     :return: a set of objects
    #     """
    #     objects = set()
    #     for ret in self.dynamic_results:
    #         if ret.hasHit() and ret.getNode().getName() in [BodyName.Traffic_cone, BodyName.Traffic_triangle]:
    #             objects.add(ret.getNode().getPythonTag(BodyName.Traffic_vehicle).kinematic_model)
    #     return objects

    def get_surrounding_vehicles_info(self, ego_vehicle, num_others: int = 4):
        from pgdrive.utils.math_utils import norm, clip
        surrounding_vehicles = list(self.get_surrounding_vehicles())
        surrounding_vehicles.sort(
            key=lambda v: norm(ego_vehicle.position[0] - v.position[0], ego_vehicle.position[1] - v.position[1])
        )
        surrounding_vehicles += [None] * num_others
        res = []
        for vehicle in surrounding_vehicles[:num_others]:
            if vehicle is not None:
                assert isinstance(vehicle, IDMVehicle), "Now PGDrive Doesn't support other vehicle type"
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

    def get_dynamic_cloud_points(self):
        return self.dynamic_cloud_points

    def get_static_cloud_points(self):
        return self.static_points

    def destroy(self):
        if self.cloud_points:
            for vis_laser in self.cloud_points:
                vis_laser.removeNode()
        self.node_path.removeNode()
        self.dynamic_elements = None
        self.dynamic_cloud_points = None
        self.static_elements = None
        self.static_points = None

    def reset(self):
        self.dynamic_elements = []
        self.dynamic_cloud_points = []
        self.static_elements = []
        self.static_points = []

    def __del__(self):
        logging.debug("Lidar is destroyed.")
