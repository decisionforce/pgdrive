from pgdrive.engine.pgdrive_engine import PGDriveEngine


def initialize_pgdrive_engine(config, agent_manager):
    cls = PGDriveEngine
    if cls.singleton is None:
        cls.singleton = cls(config, agent_manager)


def get_pgdrive_engine():
    return PGDriveEngine.singleton


def pgdrive_engine_initialized():
    return False if PGDriveEngine.singleton is None else True
