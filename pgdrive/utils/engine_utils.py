from typing import Optional

from pgdrive.engine.pgdrive_engine import PGDriveEngine
import copy

from pgdrive.utils import get_np_random


def initialize_pgdrive_engine(agent_manager):
    cls = PGDriveEngine
    if cls.singleton is None:
        assert cls.global_config is not None, "Set global config before initialization PGDriveEngine"
        cls.singleton = cls(agent_manager)
    else:
        raise PermissionError("There should be only one PGDriveEngine instance in one process")


def get_pgdrive_engine():
    return PGDriveEngine.singleton


def pgdrive_engine_initialized():
    return False if PGDriveEngine.singleton is None else True


def close_pgdrive_engine():
    if PGDriveEngine.singleton is not None:
        PGDriveEngine.singleton.close()
        PGDriveEngine.singleton = None


def set_global_config(config):
    if PGDriveEngine.global_config is None:
        PGDriveEngine.global_config = config
    else:
        PGDriveEngine.global_config.update(config)


def get_global_config():
    return copy.copy(PGDriveEngine.global_config)


def set_global_random_seed(random_seed: Optional[int]):
    """
    Update the random seed and random engine
    All subclasses of RandomEngine will hold the same random engine, after calling this function
    :param random_seed: int, random seed
    """
    engine = get_pgdrive_engine()
    engine.seed(random_seed)
