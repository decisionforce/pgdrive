from typing import Sequence

import numpy as np

from pgdrive.utils.element import Element


class Object(Element):
    """
    Common interface for objects that appear on the road, beside vehicles.
    """
    def __init__(self, position: Sequence[float], speed: float = 0., heading: float = 0.):
        """
        :param road: the road instance where the object is placed in
        :param position: cartesian position of object in the surface
        :param speed: cartesian speed of object in the surface
        :param heading: the angle from positive direction of horizontal axis
        """
        super(Object, self).__init__()
        self.position = np.array(position, dtype=np.float)
        self.speed = speed
        self.heading = heading
        # store whether object is hit by any vehicle
        self.hit = False

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
