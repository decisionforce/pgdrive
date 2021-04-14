import copy
import logging

import numpy as np
from panda3d.bullet import BulletGhostNode, BulletSphereShape, BulletAllHitsRayResult
from panda3d.core import BitMask32, NodePath
from pgdrive import cutils
from pgdrive.constants import CamMask, CollisionGroup
from pgdrive.utils.asset_loader import AssetLoader
from pgdrive.utils.coordinates_shift import panda_position


class DistanceDetector:
    """
    It is a module like lidar, used to detect sidewalk/center line or other static things
    """
    Lidar_point_cloud_obs_dim = 240
    DEFAULT_HEIGHT = 0.2

    # for vis debug
    MARK_COLOR = (51 / 255, 221 / 255, 1)
    ANGLE_FACTOR = False

    def __init__(self, parent_node_np: NodePath, num_lasers: int = 16, distance: float = 50, enable_show=False):
        # properties
        assert num_lasers > 0
        show = enable_show and (AssetLoader.loader is not None)
        self.dim = num_lasers
        self.num_lasers = num_lasers
        self.perceive_distance = distance
        self.height = self.DEFAULT_HEIGHT
        self.radian_unit = 2 * np.pi / num_lasers
        self.start_phase_offset = 0
        self.node_path = parent_node_np.attachNewNode("Could_points")
        self._lidar_range = np.arange(0, self.num_lasers) * self.radian_unit + self.start_phase_offset

        # detection result
        self.cloud_points = np.ones((self.num_lasers, ), dtype=float)
        self.detected_objects = []

        # override these properties to decide which elements to detect and show
        self.node_path.hide(CamMask.RgbCam | CamMask.Shadow | CamMask.Shadow | CamMask.DepthCam)
        self.mask = BitMask32.bit(CollisionGroup.BrokenLaneLine)
        self.cloud_points_vis = [] if show else None
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
                self.cloud_points_vis.append(laser_np)
                ball.getChildren().reparentTo(laser_np)
            # self.node_path.flattenStrong()

    def perceive(self, vehicle_position, heading_theta, pg_physics_world, extra_filter_node: set = None):
        ret_new = self._new_perceive(vehicle_position, heading_theta, pg_physics_world, extra_filter_node)
        return ret_new

        # ret_new = copy.deepcopy(ret_new)
        # ret_old = self._old_perceive(vehicle_position, heading_theta, pg_physics_world, extra_filter_node)
        # return ret_old

        # ret_new = np.asarray(ret_new)
        # ret_old_arr = np.asarray(copy.deepcopy(ret_old))
        # np.testing.assert_almost_equal(ret_new, ret_old_arr)
        # return ret_old

    def _new_perceive(self, vehicle_position, heading_theta, pg_physics_world, extra_filter_node: set = None):
        self.cloud_points, self.detected_objects, self.cloud_points_vis = cutils.cutils_perceive(
            cloud_points=self.cloud_points,
            mask=self.mask,
            lidar_range=self._lidar_range,
            perceive_distance=self.perceive_distance,
            heading_theta=heading_theta,
            vehicle_position_x=vehicle_position[0],
            vehicle_position_y=vehicle_position[1],
            num_lasers=self.num_lasers,
            height=self.height,
            pg_physics_world=pg_physics_world,
            extra_filter_node=extra_filter_node,
            cloud_points_vis=self.cloud_points_vis,
            ANGLE_FACTOR=self.ANGLE_FACTOR,
            MARK_COLOR=self.MARK_COLOR
        )
        return self.cloud_points

    def _old_perceive(self, vehicle_position, heading_theta, pg_physics_world, extra_filter_node: set = None):
        """
        Call me to update the perception info
        """
        # coordinates problem here! take care
        extra_filter_node = extra_filter_node or set()
        pg_start_position = panda_position(vehicle_position, self.height)

        # init
        self.cloud_points.fill(1.0)
        self.detected_objects = []

        # lidar calculation use pg coordinates
        mask = self.mask
        laser_heading = self._lidar_range + heading_theta
        point_x = self.perceive_distance * np.cos(laser_heading) + vehicle_position[0]
        point_y = self.perceive_distance * np.sin(laser_heading) + vehicle_position[1]

        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        for laser_index in range(self.num_lasers):
            # # coordinates problem here! take care
            laser_end = panda_position((point_x[laser_index], point_y[laser_index]), self.height)
            result = pg_physics_world.rayTestClosest(pg_start_position, laser_end, mask)
            node = result.getNode()
            hits = None
            if node in extra_filter_node:
                # Fall back to all tests.
                results: BulletAllHitsRayResult = pg_physics_world.rayTestAll(pg_start_position, laser_end, mask)
                hits = results.getHits()
                hits = sorted(hits, key=lambda ret: ret.getHitFraction())
                for result in hits:
                    if result.getNode() in extra_filter_node:
                        continue
                    self.detected_objects.append(result)
                    self.cloud_points[laser_index] = result.getHitFraction()
                    break
            else:
                self.cloud_points[laser_index] = result.getHitFraction()
                if node:
                    self.detected_objects.append(result)
                    hits = result.hasHit()

            # update vis
            if self.cloud_points_vis is not None:
                self.cloud_points_vis[laser_index].setPos(result.getHitPos() if hits else laser_end)
                f = laser_index / self.num_lasers if self.ANGLE_FACTOR else 1
                self.cloud_points_vis[laser_index].setColor(
                    f * self.MARK_COLOR[0], f * self.MARK_COLOR[1], f * self.MARK_COLOR[2]
                )
        return self.cloud_points

    def get_cloud_points(self):
        return self.cloud_points.tolist()

    def get_detected_objects(self):
        return self.detected_objects

    def destroy(self):
        if self.cloud_points_vis:
            for vis_laser in self.cloud_points_vis:
                vis_laser.removeNode()
        self.node_path.removeNode()
        self.cloud_points = None
        self.detected_objects = None

    def set_start_phase_offset(self, angle: float):
        """
        Change the start phase of lidar lasers
        :param angle: phasse offset in [degree]
        :return: None
        """
        self.start_phase_offset = np.deg2rad(angle)
        self._lidar_range = np.arange(0, self.num_lasers) * self.radian_unit + self.start_phase_offset

    def __del__(self):
        logging.debug("Lidar is destroyed.")


class SideDetector(DistanceDetector):
    def __init__(self, parent_node_np: NodePath, num_lasers: int = 2, distance: float = 50, enable_show=False):
        super(SideDetector, self).__init__(parent_node_np, num_lasers, distance, enable_show)
        self.set_start_phase_offset(90)
        self.node_path.hide(CamMask.RgbCam | CamMask.Shadow | CamMask.Shadow | CamMask.DepthCam)
        self.mask = BitMask32.bit(CollisionGroup.ContinuousLaneLine)


class LaneLineDetector(SideDetector):
    MARK_COLOR = (1, 77 / 255, 77 / 255)

    def __init__(self, parent_node_np: NodePath, num_lasers: int = 2, distance: float = 50, enable_show=False):
        super(SideDetector, self).__init__(parent_node_np, num_lasers, distance, enable_show)
        self.set_start_phase_offset(90)
        self.node_path.hide(CamMask.RgbCam | CamMask.Shadow | CamMask.Shadow | CamMask.DepthCam)
        self.mask = BitMask32.bit(CollisionGroup.ContinuousLaneLine) | BitMask32.bit(CollisionGroup.BrokenLaneLine)
