from typing import Sequence
import numpy as np
from pgdrive.utils.coordinates_shift import panda_position, panda_heading
from panda3d.core import NodePath
from pgdrive.pg_config.body_name import BodyName
from panda3d.bullet import BulletRigidBodyNode, BulletCylinderShape
from pgdrive.utils.element import Element
from pgdrive.utils.asset_loader import AssetLoader


class Object(Element):
    """
    Common interface for objects that appear on the road, beside vehicles.
    """
    NAME = None
    RADIUS = 0.25
    HEIGHT = 1.2

    def __init__(self, position: Sequence[float], heading: float = 0.):
        """
        :param position: cartesian position of object in the surface
        :param speed: cartesian speed of object in the surface
        :param heading: the angle from positive direction of horizontal axis
        """
        assert self.NAME is not None, "Assign a name for this class for finding it easily"
        super(Object, self).__init__()
        self.position = position
        self.heading = heading / np.pi * 180

    @classmethod
    def make_on_lane(cls, lane, longitudinal: float, lateral: float):
        """
        Create an object on a given lane at a longitudinal position.

        :param lane: the lane to spawn object
        :param longitudinal: longitudinal position along the lane
        :param lateral: lateral position
        :return: An object with at the specified position
        """
        return cls(lane.position(longitudinal, lateral), lane.heading_at(longitudinal))

    @classmethod
    def type(cls):
        return cls.__subclasses__()


class TrafficCone(Object):
    """Placed near the construction section to indicate that traffic is prohibited"""

    NAME = BodyName.Traffic_cone

    def __init__(self, position: Sequence[float], heading: float = 0.):
        super(TrafficCone, self).__init__(position, heading)
        body_node = BulletRigidBodyNode(self.NAME)
        body_node.addShape(BulletCylinderShape(self.RADIUS, self.HEIGHT))
        self.node_path: NodePath = NodePath(body_node)
        self.node_path.setPos(panda_position(self.position, self.HEIGHT / 2))
        self.dynamic_nodes.append(body_node)
        model = self.loader.loadModel(AssetLoader.file_path("models", "traffic_cone", "scene.gltf"))
        model.setScale(0.02)
        model.setPos(0, 0, -self.HEIGHT / 2)
        model.reparentTo(self.node_path)

        self.node_path.setH(panda_heading(self.heading))


class TrafficTriangle(Object):
    """Placed behind the vehicle when it breaks down"""

    NAME = BodyName.Traffic_triangle

    def __init__(self, position: Sequence[float], heading: float = 0.):
        super(TrafficTriangle, self).__init__(position, heading)
        body_node = BulletRigidBodyNode(self.NAME)
        body_node.addShape(BulletCylinderShape(self.RADIUS, self.HEIGHT))
        self.node_path: NodePath = NodePath(body_node)
        self.node_path.setPos(panda_position(self.position, self.HEIGHT / 2))
        self.dynamic_nodes.append(body_node)
        model = self.loader.loadModel(AssetLoader.file_path("models", "warning", "warning.gltf"))
        model.setScale(0.02)
        model.setH(-90)
        model.setPos(0, 0, -self.HEIGHT / 2)
        model.reparentTo(self.node_path)

        self.node_path.setH(panda_heading(self.heading))
