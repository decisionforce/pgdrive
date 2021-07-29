import copy
import logging
from typing import Optional

from pgdrive.engine.base_engine import BaseEngine


def initialize_pgdrive_engine(env_global_config, agent_manager):
    cls = BaseEngine
    if cls.singleton is None:
        # assert cls.global_config is not None, "Set global config before initialization BaseEngine"
        cls.singleton = cls(env_global_config, agent_manager)
    else:
        raise PermissionError("There should be only one BaseEngine instance in one process")


def get_pgdrive_engine():
    return BaseEngine.singleton


def pgdrive_engine_initialized():
    return False if BaseEngine.singleton is None else True


def close_pgdrive_engine():
    if BaseEngine.singleton is not None:
        BaseEngine.singleton.close()
        BaseEngine.singleton = None


def get_global_config():
    engine = get_pgdrive_engine()
    return copy.copy(engine.global_config)


def set_global_random_seed(random_seed: Optional[int]):
    """
    Update the random seed and random engine
    All subclasses of RandomEngine will hold the same random engine, after calling this function
    :param random_seed: int, random seed
    """
    engine = get_pgdrive_engine()
    if engine is not None:
        engine.seed(random_seed)
    else:
        logging.warning("BaseEngine is not launched, fail to sync seed to engine!")
