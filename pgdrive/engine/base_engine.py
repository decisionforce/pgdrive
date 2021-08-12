import logging
import time
from typing import Callable, Optional, Union, List
from collections import OrderedDict
from typing import Dict, AnyStr
from pgdrive.base_class.randomizable import Randomizable
import numpy as np

from pgdrive.engine.core.engine_core import EngineCore
from pgdrive.engine.interface import Interface
from pgdrive.engine.scene_cull import SceneCull
from pgdrive.manager.base_manager import BaseManager

logger = logging.getLogger(__name__)


class BaseEngine(EngineCore, Randomizable):
    """
    Due to the feature of Panda3D, BaseEngine should only be created once(Singleton Pattern)
    PGWorld is a pure game engine, which is not task-specific, while BaseEngine connects the
    driving task and the game engine modified from Panda3D Engine.
    """
    singleton = None
    global_random_seed = None

    IN_REPLAY = False
    STOP_REPLAY = False

    def __init__(self, global_config):
        EngineCore.__init__(self, global_config)
        Randomizable.__init__(self, self.global_random_seed)
        BaseEngine.singleton = self
        self.interface = Interface(self)

        # managers
        self.task_manager = self.taskMgr  # use the inner TaskMgr of Panda3D as PGDrive task manager
        self._managers = OrderedDict()

        # for recovering, they can not exist together
        # TODO new record/replay
        self.record_episode = False
        self.replay_system = None
        self.record_system = None
        self.accept("s", self._stop_replay)

        # cull scene
        self.cull_scene = self.global_config["cull_scene"]

        # add camera or not
        self.main_camera = self.setup_main_camera()

        self._spawned_objects = dict()
        self._object_policies = dict()
        self._object_tasks = dict()

        # store external actions
        self.external_actions = None

    def add_policy(self, object_id, policy):
        self._object_policies[object_id] = policy

    def add_task(self, object_id, task):
        self._object_tasks[object_id] = task

    def get_policy(self, object_id):
        """
        Return policy of specific object with id
        :param object_id: a filter function, only return objects satisfying this condition
        :return: policy
        """
        assert object_id in self._object_policies, "Can not find the policy for object(id: {})".format(object_id)
        return self._object_policies[object_id]

    def get_task(self, object_id):
        """
        Return task of specific object with id
        :param object_id: a filter function, only return objects satisfying this condition
        :return: task
        """
        assert object_id in self._object_tasks, "Can not find the task for object(id: {})".format(object_id)
        return self._object_tasks[object_id]

    def has_policy(self, object_id):
        return True if object_id in self._object_policies else False

    def has_task(self, object_id):
        return True if object_id in self._object_tasks else False

    def spawn_object(self, object_class, pbr_model=True, **kwargs):
        """
        Call this func to spawn one object
        :param object_class: object class
        :param pbr_model:
        :param kwargs: class init parameters
        :return: object spawned
        """
        if "random_seed" not in kwargs:
            kwargs["random_seed"] = self.randint()
        obj = object_class(**kwargs)
        self._spawned_objects[obj.id] = obj
        obj.attach_to_world(self.pbr_worldNP if pbr_model else self.worldNP, self.physics_world)
        return obj

    def get_objects(self, filter: Optional[Union[Callable, List]] = None):
        """
        Return objects spawned, default all objects. Filter_func will be applied on all objects.
        It can be a id list or a function
        Since we don't expect a iterator, and the number of objects is not so large, we don't use built-in filter()
        :param filter: a filter function, only return objects satisfying this condition
        :return: return all objects or objects satisfying the filter_func
        """
        if filter is None:
            return self._spawned_objects
        elif isinstance(filter, list):
            return {id: self._spawned_objects[id] for id in filter}
        elif callable(filter):
            res = dict()
            for id, obj in self._spawned_objects.items():
                if filter(obj):
                    res[id] = obj
            return res
        else:
            raise ValueError("filter should be a list or a function")

    def clear_objects(self, filter: Optional[Union[Callable, List]] = None):
        """
        Destroy all self-generated objects or objects satisfying the filter condition
        Since we don't expect a iterator, and the number of objects is not so large, we don't use built-in filter()
        """
        if filter is None:
            exclude_objects = self._spawned_objects
        elif isinstance(filter, list):
            exclude_objects = {id: self._spawned_objects[id] for id in filter}
        elif callable(filter):
            exclude_objects = dict()
            for id, obj in self._spawned_objects.items():
                if filter(obj):
                    exclude_objects[id] = obj
        else:
            raise ValueError("filter should be a list or a function")
        for id, obj in exclude_objects.items():
            self._spawned_objects.pop(id)
            if id in self._object_tasks:
                self._object_tasks.pop(id)
            if id in self._object_policies:
                policy = self._object_policies.pop(id)
                policy.destroy()
            obj.destroy()

    def reset(self):
        """
        For garbage collecting using, ensure to release the memory of all traffic vehicles
        """
        if self.global_config["debug_physics_world"]:
            self.addTask(self.report_body_nums, "report_num")

        self._episode_start_time = time.time()

        for manager in self._managers.values():
            manager.before_reset()
        for manager in self._managers.values():
            manager.reset()
        for manager in self._managers.values():
            manager.after_reset()

        if self.main_camera is not None:
            self.main_camera.reset()

    def before_step(self, external_actions: Dict[AnyStr, np.array]):
        """
        Entities make decision here, and prepare for step
        All entities can access this global manager to query or interact with others
        :param external_actions: Dict[agent_id:action]
        :return:
        """
        step_infos = {}
        self.external_actions = external_actions
        for manager in self._managers.values():
            step_infos.update(manager.before_step())
        return step_infos

    def step(self, step_num: int = 1) -> None:
        """
        Step the dynamics of each entity on the road.
        :param step_num: Decision of all entities will repeat *step_num* times
        """
        engine = self
        for i in range(step_num):
            # simulate or replay
            if self.replay_system is None:
                # not in replay mode
                for manager in self._managers.values():
                    manager.step()
                engine.step_physics_world()
            else:
                if not self.STOP_REPLAY:
                    self.replay_system.replay_frame(self.agents, i == step_num - 1)
            # # record every step
            # if self.record_system is not None:
            #     # didn't record while replay
            #     frame_state = self.traffic_manager.get_global_states()
            #     self.record_system.record_frame(frame_state)

            if engine.force_fps.real_time_simulation and i < step_num - 1:
                # insert frame to render in min step_size
                engine.task_manager.step()
        #  panda3d render and garbage collecting loop
        engine.task_manager.step()

    def after_step(self) -> Dict:
        """
        Update states after finishing movement
        :return: if this episode is done
        """
        step_infos={}
        for manager in self._managers.values():
            step_infos.update(manager.after_step())
        # cull distant blocks
        # poses = [v.position for v in self.agent_manager.active_agents.values()]
        # if self.cull_scene and False:
        #     # TODO use a for loop
        #     SceneCull.cull_distant_blocks(self, self.current_map.blocks, poses, self.global_config["max_distance"])
        #
        #     SceneCull.cull_distant_traffic_vehicles(
        #         self, self.traffic_manager.traffic_vehicles, poses, self.global_config["max_distance"]
        #     )
        #     SceneCull.cull_distant_objects(self, self.object_manager.objects, poses, self.global_config["max_distance"])
        self.interface.after_step()
        return step_infos

    def dump_episode(self) -> None:
        """Dump the data of an episode."""
        assert self.record_system is not None
        return self.record_system.dump_episode()

    def close(self):
        """
        Note:
        Instead of calling this func directly, close Engine by using engine_utils.close_engine
        """
        if self.main_camera is not None:
            self.main_camera.destroy()
        if len(self._managers) > 0:
            for name, manager in self._managers.items():
                setattr(self, name, None)
                if manager is not None:
                    manager.destroy()
        self.interface.destroy()
        self.clear_world()
        self.close_world()

    def __del__(self):
        logging.debug("{} is destroyed".format(self.__class__.__name__))

    def _stop_replay(self):
        if not self.IN_REPLAY:
            return
        self.STOP_REPLAY = not self.STOP_REPLAY

    def register_manager(self, manger_name: str, manager: BaseManager):
        """
        Add a manager to BaseEngine, then all objects can communicate with this class
        :param manger_name: name shouldn't exist in self._managers and not be same as any class attribute
        :param manager: subclass of BaseManager
        """
        assert manger_name not in self._managers, "Manager already exists in BaseEngine"
        assert not hasattr(self, manger_name), "Manager name can not be same as the attribute in BaseEngine"
        self._managers[manger_name] = manager
        self._managers.move_to_end(manger_name)
        setattr(self, manger_name, manager)

    def seed(self, random_seed):
        self.global_random_seed = random_seed
        super(BaseEngine, self).seed(random_seed)
        for mgr in self._managers.values():
            mgr.seed(random_seed)

    @property
    def current_map(self):
        return self.map_manager.current_map

    @property
    def current_track_vehicle(self):
        if self.main_camera is not None:
            return self.main_camera.current_track_vehicle
        else:
            return None

    @property
    def agents(self):
        return {k: v for k, v in self.agent_manager.active_agents.items()}

    def setup_main_camera(self):
        from pgdrive.engine.core.chase_camera import MainCamera
        if self.global_config["use_render"] or self.global_config["offscreen_render"]:
            return MainCamera(self, self.global_config["camera_height"], self.global_config["camera_dist"])
        else:
            return None
