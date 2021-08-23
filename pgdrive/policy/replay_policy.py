from pgdrive.policy.base_policy import BasePolicy

class ReplayPolicy(BasePolicy):
    def __init__(self, control_object):
        # Since control object may change
        # TODO: pass in the vehicle id or the whole trajectory
        super(ReplayPolicy, self).__init__(control_object=control_object)

    def act(self, *args, **kwargs):
        pos = self.control_object.last_position
        self.control_object.set_position([pos[0]-0.01, pos[1]+0.01])

        return [0, 0]