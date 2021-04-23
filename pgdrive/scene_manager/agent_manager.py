from gym.spaces import Box


class AgentManager:
    """
    This class maintain the relationship between active agents in the environment with the underlying instance
    of objects.

    Note:
    agent name: Agent name that exists in the environment, like agent0, agent1, ....
    object name: The unique name for each object, typically be random string.
    """

    def __init__(self, init_observations, never_allow_respawn, debug=False):
        # when new agent joins in the game, we only change this two maps.
        self.agent_to_object = {}
        self.object_to_agent = {}

        # BaseVehicles which can be controlled by policies when env.step() called
        self.active_object = {}

        # BaseVehicles which can be respawned
        self.pending_object = {}

        # Dict[object_id: value], init for **only** once after spawning vehicle
        self.observations = {}
        self.observation_spaces = {}
        self.action_spaces = {}

        self.next_agent_count = 0
        self.allow_respawn = True if not never_allow_respawn else False
        self.never_allow_respawn = never_allow_respawn
        self._debug = debug
        self._fake_init(init_observations)

    def _fake_init(self, init_observations):
        """
        For getting env.observation_space/action_space before making vehicles
        """
        self.agent_to_object = {k: k for k in init_observations.keys()}  # no target vehicles created, fake init
        self.object_to_agent = {k: k for k in init_observations.keys()}  # no target vehicles created, fake init
        self.observations = init_observations  # fake init

    def reset(self, vehicles, observation_spaces, action_spaces, observations):
        self.agent_to_object = {k: v.name for k, v in vehicles.items()}
        self.object_to_agent = {v.name: k for k, v in vehicles.items()}
        self.active_object = {v.name: v for v in vehicles.values()}
        self.next_agent_count = len(vehicles)
        self.observations = {vehicles[k].name: v for k, v in observations.items()}
        self.observation_spaces = {vehicles[k].name: v for k, v in observation_spaces.items()}
        for o in observation_spaces.values():
            assert isinstance(o, Box)
        self.action_spaces = {vehicles[k].name: v for k, v in action_spaces.items()}
        for o in action_spaces.values():
            assert isinstance(o, Box)
        self.pending_object = {}
        self.allow_respawn = True if not self.never_allow_respawn else False

    def finish(self, agent_name):
        vehicle_name = self.agent_to_object[agent_name]
        v = self.active_object.pop(vehicle_name)
        v.chassis_np.node().setStatic(True)
        assert vehicle_name not in self.active_object
        self.pending_object[vehicle_name] = v
        self._check()

    def _check(self):
        if self._debug:
            current_keys = sorted(list(self.pending_object.keys()) + list(self.active_object.keys()))
            exist_keys = sorted(list(self.object_to_agent.keys()))
            assert current_keys == exist_keys, "You should confirm_respawn() after request for propose_new_vehicle()!"

    def propose_new_vehicle(self):
        self._check()
        if len(self.pending_object) > 0:
            obj_name = list(self.pending_object.keys())[0]
            self._check()
            v = self.pending_object.pop(obj_name)
            v.prepare_step([0, -1])
            v.chassis_np.node().setStatic(False)
            return self.allow_respawn, dict(
                vehicle=v,
                observation=self.observations[obj_name],
                observation_space=self.observation_spaces[obj_name],
                action_space=self.action_spaces[obj_name],
                old_name=self.object_to_agent[obj_name],
                new_name="agent{}".format(self.next_agent_count)
            )
        return None, None

    def confirm_respawn(self, success: bool, vehicle_info):
        vehicle = vehicle_info['vehicle']
        if success:
            vehicle.set_static(False)
            self.next_agent_count += 1
            self.active_object[vehicle.name] = vehicle
            self.object_to_agent[vehicle.name] = vehicle_info["new_name"]
            self.agent_to_object.pop(vehicle_info["old_name"])
            self.agent_to_object[vehicle_info["new_name"]] = vehicle.name
        else:
            vehicle.set_static(True)
            self.pending_object[vehicle.name] = vehicle
        self._check()

    def set_allow_respawn(self, flag: bool):
        if self.never_allow_respawn:
            self.allow_respawn = False
        else:
            self.allow_respawn = flag

    def _translate(self, d):
        return {self.object_to_agent[k]: v for k, v in d.items()}

    def get_vehicle_list(self):
        return list(self.active_object.values()) + list(self.pending_object.values())

    def get_observations(self):
        return list(self.observations.values())

    def get_observation_spaces(self):
        return list(self.observation_spaces.values())

    def get_action_spaces(self):
        return list(self.action_spaces.values())
