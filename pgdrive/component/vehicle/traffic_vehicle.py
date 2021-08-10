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

    def __init__(self,vehicle_config, random_seed=None):
        """
        A traffic vehicle class.
        """
        engine = get_engine()
        config = copy.copy(engine.global_config["vehicle_config"])
        config.update(vehicle_config)
        super(TrafficVehicle, self).__init__(config, random_seed=random_seed)
        self.step(0.01, {"steering": 0, "acceleration": 0})

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

    def before_step(self):
        pass

    def step(self, dt, action=None):
        return
        if self.break_down:
            return
        if action is None:
            action = {"steering": 0, "acceleration": 0}
        self.kinematic_model.step(dt, action)

        position = panda_position(self.kinematic_model.position, 0)
        self.origin.setPos(position)
        heading = np.rad2deg(panda_heading(self.kinematic_model.heading))
        self.origin.setH(heading - 90)

    def after_step(self):
        pass

    @property
    def out_of_road(self):
        return not self.on_lane

    def need_remove(self):
        return True

    def reset(self):
        self.step(0.01, {"steering": 0, "acceleration": 0})

    def set_position(self, position, height=0.4):
        """
        Should only be called when restore traffic from episode data
        :param position: 2d array or list
        :return: None
        """
        self.origin.setPos(panda_position(position, 0))

    def set_heading(self, heading_theta) -> None:
        """
        Should only be called when restore traffic from episode data
        :param heading_theta: float in rad
        :return: None
        """
        self.origin.setH(panda_heading(heading_theta * 180 / np.pi))

    def get_state(self):
        return {"heading": self.heading, "position": self.position, "done": self.out_of_road}

    def set_state(self, state: dict):
        self.set_heading(state["heading"])
        self.set_position(state["position"])

    @property
    def heading(self):
        return self.heading_theta
    # @property
    # def heading(self):
    #     return self.kinematic_model.heading
    #
    # @property
    # def heading_theta(self):
    #     return self.kinematic_model.heading_theta
    #
    # @property
    # def position(self):
    #     return self.kinematic_model.position.tolist()
    #
    # @property
    # def speed(self):
    #     return self.kinematic_model.speed


    def __del__(self):
        super(TrafficVehicle, self).__del__()

    def set_break_down(self, break_down=True):
        self.break_down = break_down
