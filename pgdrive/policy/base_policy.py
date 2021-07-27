class BasePolicy:
    def __init__(self, *args, **kwargs):
        pass

    def act(self, *args, **kwargs):
        raise NotImplementedError()

    def destroy(self):
        pass

    def reset(self):
        pass
