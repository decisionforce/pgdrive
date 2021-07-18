from typing import Callable, Optional
from pgdrive.utils.random import RandomEngine


class BaseManager(RandomEngine):
    """
    Managers should be created and registered after launching PGDriveEngine
    """
    def __init__(self):
        from pgdrive.utils.engine_utils import get_pgdrive_engine, pgdrive_engine_initialized
        super(BaseManager, self).__init__()
        assert pgdrive_engine_initialized(), "You should not create manager before the initialization of PGDriveEngine"
        self.pgdrive_engine = get_pgdrive_engine()
        self._spawned_objects = dict()

    def spawn_object(self, object_class, **kwargs):
        """
        Call this func to spawn one object
        :param object_class: object class
        :param kwargs: class init parameters
        :return: object spawned
        """
        obj = object_class(**kwargs)
        self._spawned_objects[obj.id] = obj
        return obj

    def get_objects(self, filter_func: Optional[Callable] = None):
        """
        Return objects spawned and managed by this manager, default all objects
        Since we don't expect a iterator, and the number of objects is not so large, we don't use built-in filter()
        :param filter_func: a filter function, only return objects satisfying this condition
        :return: return all objects or objects satisfying the filter_func
        """
        res = dict()
        for id, obj in self._spawned_objects.items():
            if filter_func is None or filter_func(obj):
                res[id] = obj
        return res

    def clear_objects(self, filter_func: Optional[Callable] = None):
        """
        Destroy all self-generated objects or objects satisfying the filter condition
        Since we don't expect a iterator, and the number of objects is not so large, we don't use built-in filter()
        """
        exclude = []
        for id, obj in self._spawned_objects.items():
            if filter_func is None or filter_func(obj):
                obj.destroy()
            exclude.append(id)
        for id in exclude:
            self._spawned_objects.pop(id)

    def destroy(self):
        """
        Destroy manager
        """
        self.clear_objects()
        self._spawned_objects = None
        self.pgdrive_engine = None
