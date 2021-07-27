from pgdrive.scene_managers.base_manager import BaseManager
from pgdrive.utils import get_pgdrive_engine


class PolicyManager(BaseManager):
    def __init__(self):
        super(PolicyManager, self).__init__()
        self._agent_to_policies = {}
        self._object_to_policies = {}
        self._policy_id_to_vehicle = {}

    def reset(self):
        """
        Some policy might be stateful, for example a LSTM-based neural policy network. We need to reset the states
        of all policies here.
        """
        for p in self._spawned_objects.values():
            p.reset()

    def destroy(self):
        for p in self._spawned_objects.values():
            if hasattr(p, "destroy"):
                p.destroy()
        super(PolicyManager, self).destroy()

    def register_new_policy(self, policy_class, agent_name, vehicle, traffic_manager, *args, **kwargs):
        # e = get_pgdrive_engine()
        # TODO: We should have a general "vehicle manager". Then we can get the BaseVehicle instance from
        #  engine according to agent_name! Here is only a workaround.

        policy = self.spawn_object(policy_class, vehicle=vehicle, traffic_manager=traffic_manager, *args, **kwargs)
        policy_id = policy.name
        # TODO(pzh) Maybe we create too much dicts here?
        # self._agent_to_policies[agent_name] = policy_id
        # self._object_to_policies[policy_id] = policy
        self._policy_id_to_vehicle[policy_id] = vehicle

    def before_step(self):
        engine = get_pgdrive_engine()
        for pid, p in self._spawned_objects.items():
            v = self._policy_id_to_vehicle[pid]
            p.before_step(vehicle=v, front_vehicle=None, rear_vehicle=None, current_map=engine.current_map)

    def step(self):

        # TODO remove this!
        e = get_pgdrive_engine()
        dt = e.world_config["physics_world_step_size"]
        dt /= 3.6  # 1m/s = 3.6km/h

        for pid, p in self._spawned_objects.items():
            v = self._policy_id_to_vehicle[pid]
            p.step(dt)

    def after_step(self, *args, **kwargs):
        for pid, p in self._spawned_objects.items():
            v = self._policy_id_to_vehicle[pid]
            p.after_step()
