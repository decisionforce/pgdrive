from typing import Union
import copy

import numpy as np

from pgdrive.component.highway_vehicle.behavior import IDMVehicle
from pgdrive.component.lane.circular_lane import CircularLane
from pgdrive.component.lane.straight_lane import StraightLane
from pgdrive.component.vehicle.base_vehicle import BaseVehicle
from pgdrive.constants import CollisionGroup
from pgdrive.engine.asset_loader import AssetLoader
from pgdrive.engine.engine_utils import get_engine
from pgdrive.manager.traffic_manager import TrafficManager
from pgdrive.utils.coordinates_shift import panda_position, panda_heading


class TrafficVehicle(BaseVehicle):
    COLLISION_MASK = CollisionGroup.TrafficVehicle
    HEIGHT = 1.8
    LENGTH = 4
    WIDTH = 2
    path = None
    break_down = False
    model_collection = {}  # save memory, load model once

    def __init__(self, vehicle_config, random_seed=None):
        """
        A traffic vehicle class.
        """
        super(TrafficVehicle, self).__init__(vehicle_config, random_seed=random_seed)

    def _add_visualization(self):
        [path, scale, x_y_z_offset, H] = self.path[self.np_random.randint(0, len(self.path))]
        if self.render:
            if path not in TrafficVehicle.model_collection:
                carNP = self.loader.loadModel(AssetLoader.file_path("models", path))
                TrafficVehicle.model_collection[path] = carNP
            else:
                carNP = TrafficVehicle.model_collection[path]
            carNP.setScale(scale)
            carNP.setH(H)
            carNP.setPos(x_y_z_offset)
            carNP.instanceTo(self.origin)

    def set_break_down(self, break_down=True):
        self.break_down = break_down
