from pgdrive.envs import PGDriveEnvV2
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
    env = PGDriveEnvV2()
    env.reset()
    try:
        v = env.vehicle
        policy = IDMPolicy(
            vehicle=v,
            traffic_manager=env.pgdrive_engine.traffic_manager,
            delay_time=1,
            random_seed=env.current_seed
        )
        action = policy.before_step(v, front_vehicle=None, rear_vehicle=None,
                                    current_map=env.pgdrive_engine.current_map)
        action = policy.step(dt=0.02)
        action = policy.after_step(v, front_vehicle=None, rear_vehicle=None, current_map=env.pgdrive_engine.current_map)
        env.pgdrive_engine.policy_manager.register_new_policy(
            IDMPolicy, v.name,
            vehicle=v,
            traffic_manager=env.pgdrive_engine.traffic_manager,
            delay_time=1,
            random_seed=env.current_seed
        )
        env.step(env.action_space.sample())
        env.reset()
    finally:
        env.close()


if __name__ == '__main__':
    test_idm_policy()
