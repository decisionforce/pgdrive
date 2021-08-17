from pgdrive.base_class.randomizable import Randomizable


class BaseManager(Randomizable):
    """
    Managers should be created and registered after launching BaseEngine
    """
    PRIORITY = 10  # the engine will call managers according to the priority

    def __init__(self):
        from pgdrive.engine.engine_utils import get_engine, engine_initialized
        assert engine_initialized(), "You should not create manager before the initialization of BaseEngine"
        self.engine = get_engine()
        Randomizable.__init__(self, self.engine.global_random_seed)
        self._spawned_objects = {}

    def before_step(self, *args, **kwargs) -> dict:
        """
        Usually used to set actions for all elements with their policies
        """
        return dict()

    def step(self, *args, **kwargs):
        """
        TODO Remove in the future?
        """
        pass

    def after_step(self, *args, **kwargs) -> dict():
        """
        Update state for this manager after system advancing dt
        """
        return dict()

    def before_reset(self):
        """
        Update episode level config to this manager and clean element or detach element
        """
        self.engine.clear_objects([object_id for object_id in self._spawned_objects])
        self._spawned_objects = {}

    def reset(self):
        """
        Generate objects according to some pre-defined rules
        """
        pass

    def after_reset(self):
        """
        Usually used to record information after all managers called reset(),
        Since reset() of managers may influence each other
        """
        pass

    def destroy(self):
        """
        Destroy manager
        """
        self.engine = None
        self._spawned_objects = None

    def spawn_objects(self, *args, **kwargs):
        object = self.spawn_objects(*args, **kwargs)
        self._spawned_objects[object.id] = object
        return object
