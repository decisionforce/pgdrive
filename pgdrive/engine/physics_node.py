"""
Physics Node is the subclass of BulletNode (BulletRigidBBodyNode/BulletGhostNode and so on)
Since callback method in BulletPhysicsEngine returns PhysicsNode class and sometimes we need to do some custom
calculation and tell Object about these results, inheriting from these BulletNode class will help communicate between
Physics Callbacks and Object class
"""

from panda3d.bullet import BulletRigidBodyNode
from pgdrive.constants import BodyName


class BaseRigidBodyNode(BulletRigidBodyNode):
    def __init__(self, base_object, type_name=None):
        node_name = base_object.name if type_name is None else type_name
        super(BaseRigidBodyNode, self).__init__(node_name)
        self.setPythonTag(node_name, self)
        self.object = base_object

    def destroy(self):
        self.object = None


# TODO inheriting from the BaseRigidBodyNode, property can be set in python object


class TrafficSignNode(BulletRigidBodyNode):
    """
    Collision Properties should place here, info here can used for collision callback
    """
    COST_ONCE = True  # cost will give at the first time

    def __init__(self, object_body_name: str):
        BulletRigidBodyNode.__init__(self, object_body_name)
        BulletRigidBodyNode.setPythonTag(self, object_body_name, self)
        self.crashed = False

class TrafficVehicleNode(BulletRigidBodyNode):

    # for lidar detection and other purposes
    def __init__(self, node_name, kinematics_model):
        BulletRigidBodyNode.__init__(self, node_name)
        TrafficVehicleNode.setPythonTag(self, BodyName.Traffic_vehicle, self)
        self.kinematic_model = kinematics_model

    def reset(self, kinematics_model):
        from pgdrive.component.highway_vehicle.behavior import IDMVehicle
        self.kinematic_model = IDMVehicle.create_from(kinematics_model)
