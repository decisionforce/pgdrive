import copy
from pgdrive.constants import DEFAULT_AGENT
from pgdrive.policy.env_input_policy import EnvInputPolicy
from pgdrive.policy.idm_policy import IDMPolicy
from pgdrive.policy.manual_control_policy import ManualControlPolicy
import logging
from typing import Dict

from gym.spaces import Box, Dict

from pgdrive.manager.base_manager import BaseManager


class AgentManager(BaseManager):
    """
    This class maintain the relationship between active agents in the environment with the underlying instance
    of objects.

    Note:
    agent name: Agent name that exists in the environment, like agent0, agent1, ....
    object name: The unique name for each object, typically be random string.
    """
    INITIALIZED = False  # when vehicles instances are created, it will be set to True

    def __init__(self, init_observations, init_action_space):
        """
        The real init is happened in self.init(), in which super().__init__() will be called
        """
        # BaseVehicles which can be controlled by policies when env.step() called
        self._active_objects = {}
        # BaseVehicles which will be recycled after the delay_done time
        self._dying_objects = {}
        self._agents_finished_this_frame = dict()  # for observation space

        self.next_agent_count = 0

        # fake init. before creating engine and vehicles, it is necessary when all vehicles re-created in runtime
        self.observations = copy.copy(init_observations)  # its value is map<agent_id, obs> before init() is called
        self._init_observations = init_observations  # map <agent_id, observation>
        # init spaces before initializing env.engine
        observation_space = {
            agent_id: single_obs.observation_space
            for agent_id, single_obs in init_observations.items()
        }
        assert isinstance(init_action_space, dict)
        assert isinstance(observation_space, dict)
        self._init_observation_spaces = observation_space
        self._init_action_spaces = init_action_space
        self.observation_spaces = copy.copy(observation_space)
        self.action_spaces = copy.copy(init_action_space)

        # this map will be override when the env.init() is first called and vehicles are made
        self._agent_to_object = {k: k for k in self.observations.keys()}  # no target vehicles created, fake init
        self._object_to_agent = {k: k for k in self.observations.keys()}  # no target vehicles created, fake init

        # get the value in init()
        self._allow_respawn = None
        self._debug = None
        self._delay_done = None
        self._infinite_agents = None

    def _get_vehicles(self, config_dict: dict):
        from pgdrive.component.vehicle.vehicle_type import random_vehicle_type, only_default_agent
        ret = {}
        p = only_default_agent if not self.engine.global_config["random_agent_model"] else None
        for agent_id, v_config in config_dict.items():
            obj = self.engine.spawn_object(random_vehicle_type(self.np_random, p), vehicle_config=v_config)
            ret[agent_id] = obj
            # note: agent.id = object id
            if self.engine.global_config["manual_control"] and self.engine.global_config["use_render"]:
                policy = ManualControlPolicy()
            else:
                policy = EnvInputPolicy()
            self.engine.add_policy(obj.id, policy)
        return ret

    def reset(self):
        """
        Agent manager is really initialized after the BaseVehicle Instances are created
        """
        if not self.INITIALIZED:
            super(AgentManager, self).__init__()
            self.INITIALIZED = True

        config = self.engine.global_config
        self._debug = config["debug"]
        self._delay_done = config["delay_done"]
        self._infinite_agents = config["num_agents"] == -1
        self._allow_respawn = config["allow_respawn"]
        self.engine.clear_objects([k for k in self._active_objects.keys()] + [k for k in self._dying_objects.keys()])
        init_vehicles = self._get_vehicles(
            config_dict=self.engine.global_config["target_vehicle_configs"] if self.engine.
            global_config["is_multi_agent"] else {DEFAULT_AGENT: self.engine.global_config["vehicle_config"]}
        )
        vehicles_created = set(init_vehicles.keys())
        vehicles_in_config = set(self._init_observations.keys())
        assert vehicles_created == vehicles_in_config, "{} not defined in target vehicles config".format(
            vehicles_created.difference(vehicles_in_config)
        )

        # it is used when reset() is called to reset its original agent_id
        self._agent_to_object = {agent_id: vehicle.name for agent_id, vehicle in init_vehicles.items()}
        self._object_to_agent = {vehicle.name: agent_id for agent_id, vehicle in init_vehicles.items()}
        self._active_objects = {v.name: v for v in init_vehicles.values()}
        self._dying_objects = {}
        self._agents_finished_this_frame = dict()

        # real init {obj_name: space} map
        self.observations = dict()
        self.observation_spaces = dict()
        self.action_spaces = dict()
        for agent_id, vehicle in init_vehicles.items():
            self.observations[vehicle.name] = self._init_observations[agent_id]
            obs_space = self._init_observation_spaces[agent_id]
            self.observation_spaces[vehicle.name] = obs_space
            if not self.engine.global_config["offscreen_render"]:
                assert isinstance(obs_space, Box)
            else:
                assert isinstance(obs_space, Dict), "Multi-agent observation should be gym.Dict"
            action_space = self._init_action_spaces[agent_id]
            self.action_spaces[vehicle.name] = action_space
            assert isinstance(action_space, Box)
        self.next_agent_count = len(init_vehicles)

    def finish(self, agent_name, ignore_delay_done=False):
        """
        ignore_delay_done: Whether to ignore the delay done. This is not required when the agent success the episode!
        """
        vehicle_name = self._agent_to_object[agent_name]
        v = self._active_objects.pop(vehicle_name)
        if (not ignore_delay_done) and (self._delay_done > 0):
            self._put_to_dying_queue(v)
        else:
            # move to invisible place
            self._remove_vehicle(v)
        self._agents_finished_this_frame[agent_name] = v.name
        self._check()

    def _check(self):
        if self._debug:
            current_keys = sorted(list(self._active_objects.keys()) + list(self._dying_objects.keys()))
            exist_keys = sorted(list(self._object_to_agent.keys()))
            assert current_keys == exist_keys, "You should confirm_respawn() after request for propose_new_vehicle()!"

    def propose_new_vehicle(self):
        # Create a new vehicle.
        agent_name = self.next_agent_id()
        next_config = self.engine.global_config["target_vehicle_configs"]["agent0"]
        vehicle = self._get_vehicles({agent_name: next_config})[agent_name]
        new_v_name = vehicle.name
        self._agent_to_object[agent_name] = new_v_name
        self._object_to_agent[new_v_name] = agent_name
        self.observations[new_v_name] = self._init_observations["agent0"]
        self.observations[new_v_name].reset(vehicle)
        self.observation_spaces[new_v_name] = self._init_observation_spaces["agent0"]
        self.action_spaces[new_v_name] = self._init_action_spaces["agent0"]
        self._active_objects[vehicle.name] = vehicle
        self._check()
        vehicle.before_step([0, 0])
        vehicle.set_static(False)
        return agent_name, vehicle

    def next_agent_id(self):
        ret = "agent{}".format(self.next_agent_count)
        self.next_agent_count += 1
        return ret

    def set_allow_respawn(self, flag: bool):
        self._allow_respawn = flag

    def before_step(self):
        # not in replay mode
        self._agents_finished_this_frame = dict()
        step_infos = {}
        for agent_id in self.active_agents.keys():
            a = self.engine.get_policy(self._agent_to_object[agent_id]).act(agent_id)
            step_infos[agent_id] = self.get_agent(agent_id).before_step(a)

        finished = set()
        for v_name in self._dying_objects.keys():
            self._dying_objects[v_name][1] -= 1
            if self._dying_objects[v_name][1] == 0:  # Countdown goes to 0, it's time to remove the vehicles!
                v = self._dying_objects[v_name][0]
                self._remove_vehicle(v)
                finished.add(v_name)
        for v_name in finished:
            self._dying_objects.pop(v_name)
        return step_infos

    def after_step(self, *args, **kwargs):
        step_infos = self.for_each_active_agents(lambda v: v.after_step())
        return step_infos

    def _translate(self, d):
        return {self._object_to_agent[k]: v for k, v in d.items()}

    def get_vehicle_list(self):
        return list(self._active_objects.values()) + [v for (v, _) in self._dying_objects.values()]

    def get_observations(self):
        ret = {
            old_agent_id: self.observations[v_name]
            for old_agent_id, v_name in self._agents_finished_this_frame.items()
        }
        for obj_id, observation in self.observations.items():
            if self.is_active_object(obj_id):
                ret[self.object_to_agent(obj_id)] = observation
        return ret

    def get_observation_spaces(self):
        ret = {
            old_agent_id: self.observation_spaces[v_name]
            for old_agent_id, v_name in self._agents_finished_this_frame.items()
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
        return True if object_name in self._active_objects.keys() else False

    @property
    def active_agents(self):
        """
        Return Map<agent_id, BaseVehicle>
        """
        return {self._object_to_agent[k]: v for k, v in self._active_objects.items()}

    @property
    def active_objects(self):
        """
        Return meta-data, a pointer, Caution !
        :return: Map<obj_name, obj>
        """
        return self._active_objects

    def get_agent(self, agent_name):
        object_name = self.agent_to_object(agent_name)
        return self.get_object(object_name)

    def get_object(self, object_name):
        if object_name in self._active_objects:
            return self._active_objects[object_name]
        elif object_name in self._dying_objects:
            return self._dying_objects[object_name]
        else:
            raise ValueError("Object {} not found!".format(object_name))

    def object_to_agent(self, obj_name):
        """
        :param obj_name: BaseVehicle name
        :return: agent id
        """
        # if obj_name not in self._active_objects.keys() and self.INITIALIZED:
        #     raise ValueError("You can not access a pending Object(BaseVehicle) outside the agent_manager!")
        return self._object_to_agent[obj_name]

    def agent_to_object(self, agent_id):
        return self._agent_to_object[agent_id]

    def destroy(self):
        # when new agent joins in the game, we only change this two maps.
        if self.INITIALIZED:
            super(AgentManager, self).destroy()
        self._agent_to_object = {}
        self._object_to_agent = {}

        # BaseVehicles which can be controlled by policies when env.step() called
        self._active_objects = {}

        # BaseVehicles which can be respawned
        self._dying_objects = {}

        # Dict[object_id: value], init for **only** once after spawning vehicle
        self.observations = {}
        self.observation_spaces = {}
        self.action_spaces = {}

        self.next_agent_count = 0
        self.INITIALIZED = False

    def _put_to_dying_queue(self, v):
        vehicle_name = v.name
        v.set_static(True)
        self._dying_objects[vehicle_name] = [v, self._delay_done]

    def _remove_vehicle(self, vehicle):
        vehicle_name = vehicle.name
        assert vehicle_name not in self._active_objects
        self.engine.clear_objects([vehicle_name])
        self._agent_to_object.pop(self._object_to_agent[vehicle_name])
        self._object_to_agent.pop(vehicle_name)

    @property
    def allow_respawn(self):
        if not self._allow_respawn:
            return False
        if len(self._active_objects) + len(self._dying_objects) < self.engine.global_config["num_agents"] \
                or self._infinite_agents:
            return True
        else:
            return False

    def for_each_active_agents(self, func, *args, **kwargs):
        """
        This func is a function that take each vehicle as the first argument and *arg and **kwargs as others.
        """
        assert len(self.active_agents) > 0, "Not enough vehicles exist!"
        ret = dict()
        for k, v in self.active_agents.items():
            ret[k] = func(v, *args, **kwargs)
        return ret
