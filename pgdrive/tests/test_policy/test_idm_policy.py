from pgdrive.envs.base_env import BASE_DEFAULT_CONFIG
from pgdrive.envs.pgdrive_env import PGDriveEnvV1_DEFAULT_CONFIG
from pgdrive.policy.idm_policy import IDMPolicy
from pgdrive.scene_creator.vehicle.base_vehicle import BaseVehicle
from pgdrive.utils import PGConfig
from pgdrive.utils.engine_utils import initialize_pgdrive_engine


def _create_vehicle():
    v_config = PGConfig(BASE_DEFAULT_CONFIG["vehicle_config"]).update(PGDriveEnvV1_DEFAULT_CONFIG["vehicle_config"])
    v_config.update({"use_render": False, "use_image": False})
    config = PGConfig(BASE_DEFAULT_CONFIG)
    config.update({
        "pg_world_config": {"use_render": False, "pstats": False, "use_image": False, "debug": False},
        "vehicle_config": v_config
    })
    initialize_pgdrive_engine(config, None)
    v = BaseVehicle(vehicle_config=v_config, random_seed=0)
    return v


def test_idm_policy():
    v = _create_vehicle()
    policy = IDMPolicy(traffic_mgr=None, position=None, delay_time=1)
    action = policy.act(v)


if __name__ == '__main__':
    test_idm_policy()
