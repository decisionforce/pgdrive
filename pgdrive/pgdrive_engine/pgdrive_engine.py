from pgdrive.pgdrive_engine.world.pg_world import PGWorld
from typing import Dict
from pgdrive.scene_manager.scene_manager import SceneManager


class _PGDriveEngine(PGWorld, SceneManager):
    """
    Due to the feature of Panda3D, PGDriveEngine should only be created once(Singleton Pattern)
    PGWorld is a pure game engine, which is not task-specific, while PGDriveEngine connects the
    driving task and the game engine modified from Panda3D Engine.
    """
    singleton = None

    def __init__(self, config: Dict, agent_manager):
        self.global_config = None
        self.global_config = config
        PGWorld.__init__(self, config["pg_world_config"])
        SceneManager.__init__(self,
                              pg_world=self,
                              traffic_config={"traffic_mode": config["traffic_mode"],
                                              "random_traffic": config["random_traffic"]},
                              record_episode=config["record_episode"],
                              cull_scene=config["cull_scene"],
                              agent_manager=agent_manager)

    def __new__(cls, *args, **kwargs):
        if cls.singleton is not None:
            raise PermissionError("There should be only one PGDriveEngine instance in one process")
        else:
            return cls.__new__(*args, **kwargs)


def initialize_pgdrive_engine(config, agent_manager):
    cls = _PGDriveEngine
    if cls.singleton is None:
        cls.singleton = cls(config, agent_manager)


PGDriveEngine = _PGDriveEngine.singleton
