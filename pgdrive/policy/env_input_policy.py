from pgdrive.policy.base_policy import BasePolicy


class EnvInputPolicy(BasePolicy):

    def __init__(self, agent_id):
        # Since control object may change
        super(EnvInputPolicy, self).__init__(control_object=None)
        self.agent_id = agent_id

    def act(self):
        return self.engine.external_actions[self.agent_id]
