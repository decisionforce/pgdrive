from typing import Tuple
from panda3d.bullet import BulletBoxShape
from panda3d.bullet import BulletCylinderShape

from pgdrive.component.static_object.base_static_object import BaseStaticObject
from pgdrive.constants import BodyName
from pgdrive.constants import CollisionGroup
from pgdrive.engine.asset_loader import AssetLoader
from pgdrive.engine.physics_node import BaseRigidBodyNode
from pgdrive.utils.coordinates_shift import panda_position, panda_heading

LaneIndex = Tuple[str, str, int]


class TrafficObject(BaseStaticObject):
    """
    Common interface for objects that appear on the road, beside vehicles.
    """
    NAME = BodyName.Traffic_object
    COLLISION_GROUP = CollisionGroup.TrafficObject

    COST_ONCE = True  # cost will give at the first time

    def __init__(self, lane, longitude: float, lateral: float, random_seed):
        """
       :param lane: the lane to spawn object
        :param longitude: use to calculate cartesian position of object in the surface
        :param lateral: use to calculate the angle from positive direction of horizontal axis
        """
        position = lane.position(longitude, lateral)
        heading = lane.heading_at(longitude)
        assert self.NAME is not None, "Assign a name for this class for finding it easily"
        super(TrafficObject, self).__init__(lane, position, heading, random_seed)
        self.crashed = False


class TrafficCone(TrafficObject):
    """Placed near the construction section to indicate that traffic is prohibited"""

    RADIUS = 0.25
    HEIGHT = 1.2
    MASS = 1

    def __init__(self, lane, longitude: float, lateral: float, static: bool = False, random_seed=None):
        super(TrafficCone, self).__init__(lane, longitude, lateral, random_seed)
        self.add_body(BaseRigidBodyNode(self.name, self.NAME))
        self.body.addShape(BulletCylinderShape(self.RADIUS, self.HEIGHT))
        self.body.setIntoCollideMask(self.COLLISION_GROUP)
        self.origin.setPos(panda_position(self.position, self.HEIGHT / 2))
        self.origin.setH(panda_heading(self.heading))
        if self.render:
            model = self.loader.loadModel(AssetLoader.file_path("models", "traffic_cone", "scene.gltf"))
            model.setScale(0.02)
            model.setPos(0, 0, -self.HEIGHT / 2)
            model.reparentTo(self.origin)
        self.set_static(static)


class TrafficWarning(TrafficObject):
    """Placed behind the vehicle when it breaks down"""

    HEIGHT = 1.2
    MASS = 1
    RADIUS = 0.5

    def __init__(self, lane, longitude: float, lateral: float, static: bool = False, random_seed=None):
        super(TrafficWarning, self).__init__(lane, longitude, lateral, random_seed)
        self.add_body(BaseRigidBodyNode(self.name, self.NAME))
        self.body.addShape(BulletCylinderShape(self.RADIUS, self.HEIGHT))
        self.body.setIntoCollideMask(self.COLLISION_GROUP)
        self.origin.setPos(panda_position(self.position, self.HEIGHT / 2))
        self.origin.setH(panda_heading(self.heading))
        if self.render:
            model = self.loader.loadModel(AssetLoader.file_path("models", "warning", "warning.gltf"))
            model.setScale(0.02)
            model.setH(-90)
            model.setPos(0, 0, -self.HEIGHT / 2)
            model.reparentTo(self.origin)
        self.set_static(static)


class TrafficBarrier(TrafficObject):
    """A barrier"""

    HEIGHT = 2.0
    MASS = 10
    LENGTH = 2.0
    WIDTH = 0.3

    def __init__(self, lane, longitude: float, lateral: float, static: bool = False, random_seed=None):
        super(TrafficBarrier, self).__init__(lane, longitude, lateral, random_seed)
        self.add_body(BaseRigidBodyNode(self.name, self.NAME))
        self.body.addShape(BulletBoxShape((self.WIDTH / 2, self.LENGTH / 2, self.HEIGHT / 2)))
        self.body.setIntoCollideMask(self.COLLISION_GROUP)
        self.origin.setPos(panda_position(self.position, self.HEIGHT / 2))
        self.origin.setH(panda_heading(self.heading))
        if self.render:
            model = self.loader.loadModel(AssetLoader.file_path("models", "barrier", "scene.gltf"))
            model.setH(-90)
            model.reparentTo(self.origin)
        self.set_static(static)
