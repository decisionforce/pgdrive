from pgdrive.utils.object import Object


class BasePolicy(Object):
    def __init__(self, name=None, random_seed=None):
        super(BasePolicy, self).__init__(name=name, random_seed=random_seed)

    def destroy(self):
        pass

    def reset(self):
        pass

    def before_step(self, *args, **kwargs):
        pass

    def after_step(self, *args, **kwargs):
        pass

    def step(self, *args, **kwargs):
        pass
