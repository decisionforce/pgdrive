from pgdrive.policy.base_policy import BasePolicy
from pgdrive.engine.engine_utils import get_engine


class EnvInputPolicy(BasePolicy):

    def __init__(self, agent_id):
        super(EnvInputPolicy, self).__init__()
        self.agent_id = agent_id
        self.engine = get_engine()

    def act(self):
        return self.engine.external_actions[self.agent_id]
