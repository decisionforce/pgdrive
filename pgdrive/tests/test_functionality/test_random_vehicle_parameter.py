from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import recursive_equal


def test_random_vehicle_parameter():
    env = PGDriveEnv({
                "environment_num": 5,
                "traffic_density": .2,
                "traffic_mode": "trigger",
                "start_seed": 12,
                "random_agent_model":True
            })
    o = env.reset(force_seed=12)
    old_config_1 = env.vehicle.get_config(True)
    env.reset(force_seed=15)
    old_config_2 = env.vehicle.get_config(True)
    env.reset(force_seed=13)
    env.reset(force_seed=12)
    new_config = env.vehicle.get_config(True)
    assert recursive_equal(old_config_1, new_config)
    env.reset(force_seed=15)
    new_config = env.vehicle.get_config(True)
    assert recursive_equal(old_config_2, new_config)



