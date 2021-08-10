from pgdrive.policy.base_policy import BasePolicy



class EnvInputPolicy(BasePolicy):

    def __init__(self, agent_id):
        super(EnvInputPolicy, self).__init__()
        self.agent_id = agent_id

    def act(self):
        return self.engine.external_actions[self.agent_id]
