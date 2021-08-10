from pgdrive.base_class.randomizable import Randomizable
from pgdrive.base_class.configurable import Configurable
from pgdrive.engine.engine_utils import get_engine


class BasePolicy(Randomizable, Configurable):
    def __init__(self, random_seed=None, config=None):
        Randomizable.__init__(self,random_seed)
        Configurable.__init__(self,config)
        self.engine = get_engine()

    def act(self):
        pass
