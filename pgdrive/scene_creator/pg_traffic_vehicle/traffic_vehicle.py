from typing import Union

import numpy as np
from panda3d.bullet import BulletWorld, BulletRigidBodyNode, BulletBoxShape
from panda3d.core import BitMask32, TransformState, Point3, NodePath, Vec3

from pgdrive.pg_config.body_name import BodyName
from pgdrive.scene_creator.highway_vehicle.behavior import IDMVehicle
from pgdrive.scene_creator.lanes.circular_lane import CircularLane
from pgdrive.scene_creator.lanes.straight_lane import StraightLane
from pgdrive.scene_manager.traffic_manager import TrafficManager
from pgdrive.utils import get_np_random
from pgdrive.utils.asset_loader import AssetLoader
from pgdrive.utils.element import DynamicElement


class PgTrafficVehicleNode(BulletRigidBodyNode):

    # for lidar detection and other purposes
    def __init__(self, node_name, kinematics_model: IDMVehicle):
        BulletRigidBodyNode.__init__(self, node_name)
        PgTrafficVehicleNode.setPythonTag(self, BodyName.Traffic_vehicle, self)
        self.kinematic_model = kinematics_model

    def reset(self, kinematics_model):
        self.kinematic_model = IDMVehicle.create_from(kinematics_model)


class PgTrafficVehicle(DynamicElement):
    COLLISION_MASK = 4
    HEIGHT = 2
    path = None
    model_collection = {}  # save memory, load model once

    def __init__(self, kinematic_model: IDMVehicle, enable_reborn: bool = False, np_random=None):
        super(PgTrafficVehicle, self).__init__()
        self.vehicle_node = PgTrafficVehicleNode(BodyName.Traffic_vehicle, IDMVehicle.create_from(kinematic_model))
        chassis_shape = BulletBoxShape(Vec3(kinematic_model.LENGTH / 2, kinematic_model.WIDTH / 2, self.HEIGHT / 2))
        self.vehicle_node.addShape(chassis_shape, TransformState.makePos(Point3(0, 0, self.HEIGHT / 2 + 0.2)))
        self.vehicle_node.setMass(800.0)
        self.vehicle_node.setIntoCollideMask(BitMask32.bit(self.COLLISION_MASK))
        self.vehicle_node.setKinematic(False)
        self.vehicle_node.setStatic(True)
        self._initial_state = kinematic_model if enable_reborn else None
        self.bullet_nodes.append(self.vehicle_node)
        self.node_path = NodePath(self.vehicle_node)

        np_random = np_random or get_np_random()
        [path, scale, zoffset, H] = self.path[np_random.randint(0, len(self.path))]

        if self.render:
            if path not in PgTrafficVehicle.model_collection:
                carNP = self.loader.loadModel(AssetLoader.file_path(AssetLoader.asset_path, "models", path))
                PgTrafficVehicle.model_collection[path] = carNP
            else:
                carNP = PgTrafficVehicle.model_collection[path]
            carNP.setScale(scale)

            if path == 'new/lada/scene.gltf':
                carNP.setY(-13.5)
                carNP.setX(1)
            if path == 'new/cp/scene.gltf':
                print('fic')

            carNP.setH(H)
            carNP.setZ(zoffset)

            carNP.instanceTo(self.node_path)
        self.step(1e-1)
        # self.carNP.setQuat(LQuaternionf(np.cos(-1 * np.pi / 4), 0, 0, np.sin(-1 * np.pi / 4)))

    def prepare_step(self):
        self.vehicle_node.kinematic_model.act()

    def step(self, dt):
        self.vehicle_node.kinematic_model.step(dt)
        position = (self.vehicle_node.kinematic_model.position[0], -self.vehicle_node.kinematic_model.position[1], 0)
        self.node_path.setPos(position)
        heading = np.rad2deg(-self.vehicle_node.kinematic_model.heading)
        self.node_path.setH(heading)

    def update_state(self):
        self.vehicle_node.kinematic_model.on_state_update()

    def need_remove(self):
        if self._initial_state is not None:
            self.vehicle_node.reset(self._initial_state)
            return False
        else:
            self.vehicle_node.clearTag(BodyName.Traffic_vehicle)
            self.node_path.removeNode()
            return True

    def out_of_road(self):
        return not self.vehicle_node.kinematic_model.lane.on_lane(self.vehicle_node.kinematic_model.position, margin=2)

    def destroy(self, pg_world: BulletWorld):
        self.vehicle_node.clearTag(BodyName.Traffic_vehicle)
        super(PgTrafficVehicle, self).destroy(pg_world)

    @classmethod
    def create_random_traffic_vehicle(
        cls,
        scene: TrafficManager,
        lane: Union[StraightLane, CircularLane],
        longitude: float,
        seed=None,
        enable_lane_change: bool = True,
        enable_reborn=False
    ):
        v = IDMVehicle.create_random(scene, lane, longitude, random_seed=seed)
        v.enable_lane_change = enable_lane_change
        return cls(v, enable_reborn, np_random=v.np_random)

    def __del__(self):
        self.vehicle_node.clearTag(BodyName.Traffic_vehicle)
        super(PgTrafficVehicle, self).__del__()
