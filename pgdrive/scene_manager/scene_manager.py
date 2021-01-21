import logging
from typing import List, Tuple

from pgdrive.scene_creator.map import Map
from pgdrive.scene_manager.traffic_manager import TrafficManager
from pgdrive.scene_manager.replay_record_system import PGReplayer, PGRecorder
from pgdrive.world.pg_world import PGWorld
from pgdrive.scene_manager.traffic_manager import TrafficMode

logger = logging.getLogger(__name__)

LaneIndex = Tuple[str, str, int]
Route = List[LaneIndex]


class SceneManager:
    """Manage all traffic vehicles, and all runtime elements (in the future)"""
    def __init__(self, traffic_mode=TrafficMode.Trigger, random_traffic: bool = False, record_episode: bool = False):
        """
        :param traffic_mode: reborn/trigger mode
        :param random_traffic: if True, map seed is different with traffic manager seed
        """
        self.traffic = TrafficManager(traffic_mode, random_traffic)
        self.ego_vehicle = None
        self.map = None

        # for recovering, they can not exist together
        self.record_episode = record_episode
        self.replay_system = None
        self.record_system = None

    def reset(self, pg_world: PGWorld, map: Map, ego_vehicle, traffic_density: float, episode_data=None):
        """
        For garbage collecting using, ensure to release the memory of all traffic vehicles
        """
        self.ego_vehicle = ego_vehicle

        if self.replay_system is not None:
            self.replay_system.destroy(pg_world)
            self.replay_system = None
        if self.record_system is not None:
            self.record_system.destroy(pg_world)
            self.record_system = None

        if episode_data is None:
            self.traffic.generate(pg_world, map, [ego_vehicle], traffic_density)
        else:
            self.replay_system = PGReplayer(self.traffic, map, episode_data, pg_world)

        if pg_world.highway_render is not None:
            pg_world.highway_render.set_scene_mgr(self)
        if self.record_episode:
            if episode_data is None:
                self.record_system = PGRecorder(map, self.traffic.get_global_init_states(), self.traffic.mode)
            else:
                logging.warning("Temporally disable episode recorder, since we are replaying other episode!")

    def prepare_step(self):
        """
        Entities make decision here, and prepare for step
        All entities can access this global manager to query or interact with others
        :return: None
        """
        self.traffic.prepare_step(self)

    def step(self, dt: float) -> None:
        """
        Step the dynamics of each entity on the road.

        :param dt: timestep [s]
        """
        self.traffic.step(dt)

    def update_state(self, pg_world: PGWorld) -> bool:
        """
        Update states after finishing movement
        :param pg_world: World
        :return: if this episode is done
        """
        done = False
        done = self.traffic.update_state(pg_world) or done

        if self.replay_system is not None:
            self.replay_system.replay_frame(self.ego_vehicle, pg_world)
        elif self.record_system is not None:
            # didn't record while replay
            self.record_system.record_frame(self.traffic.get_global_states())
        return done

    def dump_episode(self) -> None:
        """Dump the data of an episode."""
        assert self.record_system is not None
        return self.record_system.dump_episode()

    def destroy(self, pg_world: PGWorld):
        self.traffic.destroy(pg_world)
        self.traffic = None
        self.map = None
        if self.record_system is not None:
            self.record_system.destroy(pg_world)
            self.record_system = None
        if self.replay_system is not None:
            self.replay_system.destroy(pg_world)
            self.replay_system = None

    def __repr__(self):
        info = "traffic:" + self.traffic.__repr__()
        return info

    def __del__(self):
        logging.debug("{} is destroyed".format(self.__class__.__name__))
