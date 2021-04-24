from gym.spaces import Box
import copy
# from pgdrive.scene_creator.vehicle.base_vehicle import BaseVehicle
from typing import List, Dict
from pgdrive.obs.observation_type import ObservationType

from typing import List, Tuple, Optional, Dict, AnyStr, Union, Callable

import numpy as np


class AgentManager:
    """
    This class maintain the relationship between active agents in the environment with the underlying instance
    of objects.

    Note:
    agent name: Agent name that exists in the environment, like agent0, agent1, ....
    object name: The unique name for each object, typically be random string.
    """
    INITIALIZED = False  # when vehicles instances are created, it will be set to True

    def __init__(self, init_observations, never_allow_respawn, debug=False):
        # when new agent joins in the game, we only change this two maps.
        self.__agent_to_object = {}
        self.__object_to_agent = {}

        # BaseVehicles which can be controlled by policies when env.step() called
        self.__active_objects = {}

        # BaseVehicles which can be respawned
        self.__pending_objects = {}

        # Dict[object_id: value], init for **only** once after spawning vehicle
        self.observations = {}
        self.observation_spaces = {}
        self.action_spaces = {}

        self.next_agent_count = 0
        self.allow_respawn = True if not never_allow_respawn else False
        self.never_allow_respawn = never_allow_respawn
        self._debug = debug

        self.__init_object_to_agent = None
        self.__agents_finished_this_frame = dict()  # for

        # fake init. before creating pg_world and vehicles, it is necessary when all vehicles re-created in runtime
        self.observations = copy.copy(init_observations)  # its value is map<agent_id, obs> before init() is called
        self.__init_observations = init_observations  # map <agent_id, observation>
        self.__init_observation_spaces = None  # map <agent_id, space>
        self.__init_action_spaces = None  # map <agent_id, space>

        # this map will be override when the env.init() is first called and vehicles are made
        self.__agent_to_object = {k: k for k in self.observations.keys()}  # no target vehicles created, fake init
        self.__object_to_agent = {k: k for k in self.observations.keys()}  # no target vehicles created, fake init

    def init_space(self, init_observation_space, init_action_space):
        """
        For getting env.observation_space/action_space before making vehicles
        """
        self.__init_observation_spaces = init_observation_space
        self.observation_spaces = copy.copy(init_observation_space)

        self.__init_action_spaces = init_action_space
        self.action_spaces = copy.copy(init_action_space)

    def init(self, init_vehicles: Dict):
        """
        Agent manager is really initialized after the BaseVehicle Instances are created
        """
        vehicles_created = set(init_vehicles.keys())
        vehicles_in_config = set(self.__init_observations.keys())
        assert vehicles_created == vehicles_in_config, "{} not defined in target vehicles config".format(
            vehicles_created.difference(vehicles_in_config)
        )

        self.INITIALIZED = True
        # it is used when reset() is called to reset its original agent_id
        self.__init_object_to_agent = {vehicle.name: agent_id for agent_id, vehicle in init_vehicles.items()}

        self.__agent_to_object = {agent_id: vehicle.name for agent_id, vehicle in init_vehicles.items()}
        self.__object_to_agent = {vehicle.name: agent_id for agent_id, vehicle in init_vehicles.items()}
        self.__active_objects = {v.name: v for v in init_vehicles.values()}
        self.__pending_objects = {}

        # real init {obj_name: space} map
        self.observations = dict()
        self.observation_spaces = dict()
        self.action_spaces = dict()
        for agent_id, vehicle in init_vehicles.items():
            self.observations[vehicle.name] = self.__init_observations[agent_id]

            obs_space = self.__init_observation_spaces[agent_id]
            self.observation_spaces[vehicle.name] = obs_space
            assert isinstance(obs_space, Box)
            action_space = self.__init_action_spaces[agent_id]
            self.action_spaces[vehicle.name] = action_space
            assert isinstance(action_space, Box)

    def reset(self):
        self.__agents_finished_this_frame = dict()
        # free them in physics world
        for v in self.__pending_objects.values():
            v.chassis_np.node().setStatic(False)

        vehicles = self.get_vehicle_list()
        assert len(vehicles) == len(self.observations) == len(self.observation_spaces) == len(self.action_spaces)
        origin_agent_id_vehicles = {self.__init_object_to_agent[v.name]: v for v in vehicles}

        self.__agent_to_object = {k: v.name for k, v in origin_agent_id_vehicles.items()}
        self.__object_to_agent = {v.name: k for k, v in origin_agent_id_vehicles.items()}
        self.next_agent_count = len(vehicles)
        self.__active_objects = {v.name: v for v in origin_agent_id_vehicles.values()}
        self.__pending_objects = {}
        self.allow_respawn = True if not self.never_allow_respawn else False

    def finish(self, agent_name):
        vehicle_name = self.__agent_to_object[agent_name]
        v = self.__active_objects.pop(vehicle_name)
        v.chassis_np.node().setStatic(True)
        assert vehicle_name not in self.__active_objects
        self.__pending_objects[vehicle_name] = v
        self.__agents_finished_this_frame[agent_name] = v.name
        self._check()

    def _check(self):
        if self._debug:
            current_keys = sorted(list(self.__pending_objects.keys()) + list(self.__active_objects.keys()))
            exist_keys = sorted(list(self.__object_to_agent.keys()))
            assert current_keys == exist_keys, "You should confirm_respawn() after request for propose_new_vehicle()!"

    def propose_new_vehicle(self):
        self._check()
        if len(self.__pending_objects) > 0:
            obj_name = list(self.__pending_objects.keys())[0]
            self._check()
            v = self.__pending_objects.pop(obj_name)
            v.prepare_step([0, -1])
            v.chassis_np.node().setStatic(False)
            self.observations[obj_name].reset(v)
            return self.allow_respawn, dict(
                vehicle=v,
                observation=self.observations[obj_name],
                observation_space=self.observation_spaces[obj_name],
                action_space=self.action_spaces[obj_name],
                old_name=self.__object_to_agent[obj_name],
                new_name="agent{}".format(self.next_agent_count)
            )
        return None, None

    def confirm_respawn(self, success: bool, vehicle_info):
        vehicle = vehicle_info['vehicle']
        if success:
            vehicle.set_static(False)
            self.next_agent_count += 1
            self.__active_objects[vehicle.name] = vehicle
            self.__object_to_agent[vehicle.name] = vehicle_info["new_name"]
            self.__agent_to_object.pop(vehicle_info["old_name"])
            self.__agent_to_object[vehicle_info["new_name"]] = vehicle.name
        else:
            vehicle.set_static(True)
            self.__pending_objects[vehicle.name] = vehicle
        self._check()

    def set_allow_respawn(self, flag: bool):
        if self.never_allow_respawn:
            self.allow_respawn = False
        else:
            self.allow_respawn = flag

    def prepare_step(self, target_actions: Dict[AnyStr, np.array]):
        """
        Entities make decision here, and prepare for step
        All entities can access this global manager to query or interact with others
        :param target_actions: Dict[agent_id:action]
        :return:
        """
        self.__agents_finished_this_frame = dict()
        object_to_agent = self.object_to_agent
        step_infos = {}
        # if self.replay_system is None:
        # not in replay mode
        for k in self.__active_objects.keys():
            a = target_actions[object_to_agent(k)]
            step_infos[object_to_agent(k)] = self.__active_objects[k].prepare_step(a)
        return step_infos

    def _translate(self, d):
        return {self.__object_to_agent[k]: v for k, v in d.items()}

    def get_vehicle_list(self):
        return list(self.__active_objects.values()) + list(self.__pending_objects.values())

    def get_observations(self):
        ret = {
            old_agent_id: self.observations[v_name]
            for old_agent_id, v_name in self.__agents_finished_this_frame.items()
        }
        for obj_id, observation in self.observations.items():
            if self.is_active_object(obj_id):
                ret[self.object_to_agent(obj_id)] = observation
        return ret

    def get_observation_spaces(self):
        ret = {
            old_agent_id: self.observation_spaces[v_name]
            for old_agent_id, v_name in self.__agents_finished_this_frame.items()
        }
        for obj_id, space in self.observation_spaces.items():
            if self.is_active_object(obj_id):
                ret[self.object_to_agent(obj_id)] = space
        return ret

    def get_action_spaces(self):
        ret = dict()
        for obj_id, space in self.action_spaces.items():
            if self.is_active_object(obj_id):
                ret[self.object_to_agent(obj_id)] = space
        return ret

    def is_active_object(self, object_name):
        if not self.INITIALIZED:
            return True
        return True if object_name in self.__active_objects.keys() else False

    @property
    def active_objects(self):
        """
        Return Map<agent_id, BaseVehicle>
        """
        return {self.__object_to_agent[k]: v for k, v in self.__active_objects.items()}

    def get_active_objects(self):
        """
        Return meta-data, a pointer, Caution !
        :return: Map<obj_name, obj>
        """
        return self.__active_objects

    @property
    def pending_objects(self):
        """
        Return Map<agent_id, BaseVehicle>
        """
        return {self.__object_to_agent[k]: v for k, v in self.__pending_objects.items()}

    def object_to_agent(self, obj_name):
        """
        :param obj_name: BaseVehicle name
        :return: agent id
        """
        if obj_name not in self.__active_objects.keys() and self.INITIALIZED:
            raise ValueError("You can not access a pending Object(BaseVehicle) outside the agent_manager!")
        return self.__object_to_agent[obj_name]

    def agent_to_object(self, agent_id):
        return self.__agent_to_object[agent_id]

    def destroy(self):
        # when new agent joins in the game, we only change this two maps.
        self.__agent_to_object = {}
        self.__object_to_agent = {}

        # BaseVehicles which can be controlled by policies when env.step() called
        self.__active_objects = {}

        # BaseVehicles which can be respawned
        self.__pending_objects = {}

        # Dict[object_id: value], init for **only** once after spawning vehicle
        self.observations = {}
        self.observation_spaces = {}
        self.action_spaces = {}

        self.next_agent_count = 0

    def update_state_for_all_target_vehicles(self, detector_mask: Union["DetectorMask", None] = None):
        step_infos = self.for_each_target_vehicle(
            lambda v: v.update_state(detector_mask=detector_mask.get_mask(v.name) if detector_mask else None)
        )
        return step_infos

    def for_each_target_vehicle(self, func):
        """Apply the func (a function take only the vehicle as argument) to each target vehicles and return a dict!"""
        assert len(self.__active_objects) > 0
        ret = dict()
        for k, v in self.__active_objects.items():
            ret[self.object_to_agent(k)] = func(v)
        return ret
