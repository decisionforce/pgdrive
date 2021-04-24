import logging
from typing import List, Tuple, Dict, Union

from pgdrive.scene_creator.map import Map
from pgdrive.scene_creator.vehicle_module.distance_detector import DetectorMask
from pgdrive.scene_manager.PGLOD import PGLOD
from pgdrive.scene_manager.object_manager import ObjectsManager
from pgdrive.scene_manager.traffic_manager import TrafficManager
from pgdrive.utils import PGConfig
from pgdrive.world.pg_world import PGWorld

logger = logging.getLogger(__name__)

LaneIndex = Tuple[str, str, int]
Route = List[LaneIndex]


class SceneManager:
    """Manage all traffic vehicles, and all runtime elements (in the future)"""

    def __init__(
            self,
            config,
            pg_world: PGWorld,
            traffic_config: Union[Dict, "PGConfig"],
            # traffic_mode=TrafficMode.Trigger,
            # random_traffic: bool = False,
            # record_episode: bool,
            cull_scene: bool,
            agent_manager: "AgentManager"
            # object_to_agent: Callable,
            # agent_to_object: Callable,
            # get_active_objects: Callable
    ):
        """
        :param traffic_mode: respawn/trigger mode
        :param random_traffic: if True, map seed is different with traffic manager seed
        """
        # scene manager control all movements in pg_world
        self.pg_world = pg_world

        self._agent_manager = agent_manager
        self._traffic_manager = self._get_traffic_manager(traffic_config)
        self._objects_manager = self._get_object_manager()

        # common variable
        # self.__target_vehicles = None
        # self.get_active_vehicles = get_active_objects
        # self.object_to_agent: Callable = object_to_agent
        # self.agent_to_object: Callable = agent_to_object
        self.map = None

        # for recovering, they can not exist together
        # self.record_episode = record_episode
        # self.replay_system: Optional[PGReplayer] = None
        # self.record_system: Optional[PGRecorder] = None

        # cull scene
        self.cull_scene = cull_scene
        self.detector_mask = None

    def _get_traffic_manager(self, traffic_config):
        return TrafficManager(traffic_config["traffic_mode"], traffic_config["random_traffic"])

    def _get_object_manager(self, object_config=None):
        return ObjectsManager()

    def reset(self, map: Map, target_vehicles: List, traffic_density: float, accident_prob: float, replaying=False):
        """
        For garbage collecting using, ensure to release the memory of all traffic vehicles
        """
        pg_world = self.pg_world
        assert isinstance(target_vehicles, list)
        # self.__target_vehicles = self.get_active_vehicles()
        self.map = map

        self._traffic_manager.reset(pg_world, map, target_vehicles, traffic_density)
        self._objects_manager.reset(pg_world, map, accident_prob)

        if self.detector_mask is not None:
            self.detector_mask.clear()

        if not replaying:
            self._objects_manager.generate(self, pg_world)
            self._traffic_manager.generate(
                pg_world=pg_world,
                map=self.map,
                target_vehicles=self._agent_manager.get_active_objects(),
                traffic_density=traffic_density
            )

        # if pg_world.highway_render is not None:
        #     pg_world.highway_render.set_scene_mgr(self)

    def setup_detector_mask(self, num_lasers, max_distance, max_span):
        self.detector_mask = DetectorMask(num_lasers=num_lasers, max_distance=max_distance, max_span=max_span)

    def prepare_step(self, replaying=False):
        """
        Entities make decision here, and prepare for step
        All entities can access this global manager to query or interact with others
        :param target_actions: Dict[agent_id:action]
        :return:
        """
        # self._traffic_manager.prepare_step(self)
        # object_to_agent = self.object_to_agent
        # step_infos = {}
        if not replaying:
            # # not in replay mode
            # for k in self.__target_vehicles.keys():
            #     a = target_actions[object_to_agent(k)]
            #     step_infos[object_to_agent(k)] = self.__target_vehicles[k].prepare_step(a)
            self._traffic_manager.prepare_step(self)
        # return {}
        # return step_infos

    def step(self, step_num: int = 1, replaying=False) -> None:
        """
        Step the dynamics of each entity on the road.
        :param pg_world: World
        :param step_num: Decision of all entities will repeat *step_num* times
        :param replaying: Whether we are replaying an episode.
        """
        pg_world = self.pg_world
        dt = pg_world.world_config["physics_world_step_size"]
        for i in range(step_num):
            if not replaying:
                # not in replay mode
                self._traffic_manager.step(dt)
                pg_world.step()
            if pg_world.force_fps.real_time_simulation and i < step_num - 1:
                # insert frame to render in min step_size
                pg_world.taskMgr.step()

        #  panda3d render and garbage collecting loop
        pg_world.taskMgr.step()

    def update_state(self, active_vehicles, replaying=False) -> Dict:
        """
        Update states after finishing movement
        :return: if this episode is done
        """
        # self.for_each_target_vehicle(lambda v: self.replay_system.replay_frame(v, self.pg_world))
        # if self.replay_system is not None:
        #     self.for_each_target_vehicle(lambda v: self.replay_system.replay_frame(v, self.pg_world))
            # self.replay_system.replay_frame(self.ego_vehicle, self.pg_world)
        # else:
        if not replaying:
            self._traffic_manager.update_state(self, self.pg_world)

        # if self.record_system is not None:
            # didn't record while replay
            # self.record_system.record_frame(self._traffic_manager.get_global_states())

        # step_infos = self.update_state_for_all_target_vehicles()

        # cull distant blocks
        poses = [v.position for v in active_vehicles.values()]
        if self.cull_scene:
            PGLOD.cull_distant_blocks(self.map.blocks, poses, self.pg_world, self.pg_world.world_config["max_distance"])
            # PGLOD.cull_distant_blocks(self.map.blocks, self.ego_vehicle.position, self.pg_world)

            if not replaying:
                # TODO add objects to replay system and add new cull method

                PGLOD.cull_distant_traffic_vehicles(
                    self._traffic_manager.traffic_vehicles, poses, self.pg_world,
                    self.pg_world.world_config["max_distance"]
                )
                PGLOD.cull_distant_objects(
                    self._objects_manager._spawned_objects, poses, self.pg_world,
                    self.pg_world.world_config["max_distance"]
                )

        return {}
        # return step_infos

    def update_detector_mask(self):
        if self.detector_mask is not None:
            # a = set([v.name for v in self._traffic_manager.vehicles])
            # b = set([v.name for v in self.__target_vehicles.values()])
            # assert b.issubset(a)  # This may only happen during episode replays!
            is_target_vehicle_dict = {
                v_obj.name: self._traffic_manager.is_target_vehicle(v_obj)
                for v_obj in self._traffic_manager.vehicles + self._objects_manager.objects
            }
            self.detector_mask.update_mask(
                position_dict={
                    v_obj.name: v_obj.position
                    for v_obj in self._traffic_manager.vehicles + self._objects_manager.objects
                },
                heading_dict={
                    v_obj.name: v_obj.heading_theta
                    for v_obj in self._traffic_manager.vehicles + self._objects_manager.objects
                },
                is_target_vehicle_dict=is_target_vehicle_dict
            )

    # def update_state_for_all_target_vehicles(self):
    #     if self.detector_mask is not None:
    #         # a = set([v.name for v in self._traffic_manager.vehicles])
    #         # b = set([v.name for v in self.__target_vehicles.values()])
    #         # assert b.issubset(a)  # This may only happen during episode replays!
    #         is_target_vehicle_dict = {
    #             v_obj.name: self._traffic_manager.is_target_vehicle(v_obj)
    #             for v_obj in self._traffic_manager.vehicles + self._objects_manager.objects
    #         }
    #         self.detector_mask.update_mask(
    #             position_dict={
    #                 v_obj.name: v_obj.position
    #                 for v_obj in self._traffic_manager.vehicles + self._objects_manager.objects
    #             },
    #             heading_dict={
    #                 v_obj.name: v_obj.heading_theta
    #                 for v_obj in self._traffic_manager.vehicles + self._objects_manager.objects
    #             },
    #             is_target_vehicle_dict=is_target_vehicle_dict
    #         )
    #     step_infos = self.for_each_target_vehicle(
    #         lambda v: v.update_state(detector_mask=self.detector_mask.get_mask(v.name) if self.detector_mask else None)
    #     )
    #     return step_infos

    # def for_each_target_vehicle(self, func):
    #     """Apply the func (a function take only the vehicle as argument) to each target vehicles and return a dict!"""
    #     assert len(self.__target_vehicles) > 0
    #     ret = dict()
    #     for k, v in self.__target_vehicles.items():
    #         ret[self.object_to_agent(k)] = func(v)
    #     return ret

    def destroy(self, pg_world: PGWorld = None):
        pg_world = self.pg_world if pg_world is None else pg_world
        self._traffic_manager.destroy(pg_world)
        self._traffic_manager = None

        self._objects_manager.destroy(pg_world)
        self._objects_manager = None

        self.map = None

    def __repr__(self):
        info = "traffic:" + self._traffic_manager.__repr__()
        return info

    def __del__(self):
        logging.debug("{} is destroyed".format(self.__class__.__name__))

    def is_target_vehicle(self, v):
        return v in self.__target_vehicles.values()

    @property
    def target_vehicles(self):
        return {self.object_to_agent(k): v for k, v in self.__target_vehicles.items()}

    @property
    def traffic_manager(self):
        return self._traffic_manager

    def destroy_detector_mask(self):
        self.detector_mask.clear()
        self.detector_mask = None
