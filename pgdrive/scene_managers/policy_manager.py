from pgdrive.scene_managers.base_manager import BaseManager


class PolicyManager(BaseManager):
    def __init__(self):
        super(PolicyManager, self).__init__()
        self._agent_to_policies = {}
        self._object_to_policies = {}

    def reset(self):
        """
        Some policy might be stateful, for example a LSTM-based neural policy network. We need to reset the states
        of all policies here.
        """
        raise NotImplementedError()

    def destroy(self):
        for p in self._spawned_objects.values():
            if hasattr(p, "destroy"):
                p.destroy()
        super(PolicyManager, self).destroy()
