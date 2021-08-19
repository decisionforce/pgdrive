from pgdrive import PGDriveEnvV2
from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import recursive_equal, norm


def test_seeding():
    env = PGDriveEnv({"environment_num": 1000})
    try:
        env.reset()
        env.seed(999)
        # assert env.engine is None
        assert env.current_seed == 999
        env.reset(force_seed=992)
        assert env.current_seed == 992
        # assert env.engine is not None
    finally:
        env.close()


def test_map_random_seeding():
    cfg_1 = {
        "environment_num": 1,
        "start_seed": 5,
    }
    cfg_2 = {
        "environment_num": 10,
        "start_seed": 5,
    }
    cfg_3 = {
        "environment_num": 100,
        "start_seed": 5,
    }
    cfg_4 = {
        "environment_num": 10,
        "start_seed": 0,
    }
    cfg_5 = {
        "environment_num": 3,
        "start_seed": 3,
    }
    map_configs = []
    for cfg in [cfg_1, cfg_2, cfg_3, cfg_4, cfg_5]:
        env = PGDriveEnv(cfg)
        try:
            env.reset(force_seed=5)
            map_configs.append(env.current_map.save_map)
        finally:
            env.close()
    for idx, map_cfg in enumerate(map_configs[:-1]):
        nxt_map_cfg = map_configs[idx + 1]
        recursive_equal(map_cfg, nxt_map_cfg)


def test_fixed_traffic():
    env = PGDriveEnvV2({
        "random_traffic": False,
        "traffic_mode": "respawn",
        # "fast": True, "use_render": True
    })
    try:
        last_pos = None
        for i in range(20):
            obs = env.reset()
            assert env.engine.traffic_manager.random_seed == env.current_seed
            new_pos = [v.position for v in env.engine.traffic_manager.vehicles]
            if last_pos is not None and len(new_pos) == len(last_pos):
                assert sum(
                    [norm(lastp[0] - newp[0], lastp[1] - newp[1]) <= 1e-3 for lastp, newp in zip(last_pos, new_pos)]
                ), [(lastp, newp) for lastp, newp in zip(last_pos, new_pos)]
            last_pos = new_pos
    finally:
        env.close()


def test_random_traffic():
    env = PGDriveEnvV2(
        {
            "random_traffic": True,
            "traffic_mode": "respawn",
            "traffic_density": 0.3,
            "start_seed": 5,

            # "fast": True, "use_render": True
        }
    )
    has_traffic = False
    try:
        last_pos = None
        for i in range(20):
            obs = env.reset(force_seed=5)
            assert env.engine.traffic_manager.random_traffic
            new_pos = [v.position for v in env.engine.traffic_manager.traffic_vehicles]
            if len(new_pos) > 0:
                has_traffic = True
            if last_pos is not None and len(new_pos) == len(last_pos):
                assert sum(
                    [norm(lastp[0] - newp[0], lastp[1] - newp[1]) >= 0.5 for lastp, newp in zip(last_pos, new_pos)]
                ), [(lastp, newp) for lastp, newp in zip(last_pos, new_pos)]
            last_pos = new_pos
        assert has_traffic
    finally:
        env.close()


def test_random_lane_width():
    env = PGDriveEnv(
        {
            "environment_num": 5,
            "traffic_density": .2,
            "traffic_mode": "trigger",
            "start_seed": 12,
            "random_lane_width": True,
            "load_map_from_json": False
        }
    )
    try:
        o = env.reset(force_seed=12)
        old_config_1 = env.vehicle.lane.width
        env.reset(force_seed=15)
        old_config_2 = env.vehicle.lane.width
        env.reset(force_seed=13)
        env.reset(force_seed=12)
        new_config = env.vehicle.lane.width
        assert old_config_1 == new_config
        env.reset(force_seed=15)
        new_config = env.vehicle.lane.width
        assert old_config_2 == new_config
        assert old_config_1 != old_config_2
    finally:
        env.close()


def test_random_lane_num():
    env = PGDriveEnv(
        {
            "environment_num": 5,
            "traffic_density": .2,
            "traffic_mode": "trigger",
            "start_seed": 12,
            "load_map_from_json": False,
            "random_lane_num": True,
        }
    )
    try:
        o = env.reset(force_seed=12)
        old_config_1 = env.vehicle.navigation.get_current_lane_num()
        env.reset(force_seed=15)
        old_config_2 = env.vehicle.navigation.get_current_lane_num()
        env.reset(force_seed=13)
        env.reset(force_seed=12)
        new_config = env.vehicle.navigation.get_current_lane_num()
        assert old_config_1 == new_config
        env.reset(force_seed=15)
        new_config = env.vehicle.navigation.get_current_lane_num()
        assert old_config_2 == new_config
        env.close()
        env.reset(force_seed=12)
        assert old_config_1 == env.vehicle.navigation.get_current_lane_num()
        env.reset(force_seed=15)
        assert old_config_2 == env.vehicle.navigation.get_current_lane_num()
    finally:
        env.close()


def test_random_vehicle_parameter():
    env = PGDriveEnv(
        {
            "environment_num": 5,
            "traffic_density": .2,
            "traffic_mode": "trigger",
            "start_seed": 12,
            "random_agent_model": True
        }
    )
    try:
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
    finally:
        env.close()


if __name__ == '__main__':
    # test_map_random_seeding()
    test_seeding()
