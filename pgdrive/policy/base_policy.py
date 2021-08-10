from pgdrive.base_class.randomizable import Randomizable
from pgdrive.base_class.configurable import Configurable


class BasePolicy(Randomizable, Configurable):
    def __init__(self, random_seed=None, config=None):
        Randomizable.__init__(self,random_seed)
        Configurable.__init__(self,config)

    def act(self):
        pass
