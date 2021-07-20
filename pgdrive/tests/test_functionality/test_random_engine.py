from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import recursive_equal


def test_seeding():
    env = PGDriveEnv({"environment_num":1000})
    try:
        env.seed(999)
        assert env.pgdrive_engine is None
        assert env.current_seed == 999
        env.reset(force_seed=999)
        assert env.current_seed == 999
        assert env.pgdrive_engine is not None
    finally:
        env.close()


def test_map_random_seeding():
    cfg_1 = {"environment_num": 1,
             "start_seed": 5, }
    cfg_2 = {"environment_num": 10,
             "start_seed": 5, }
    cfg_3 = {"environment_num": 100,
             "start_seed": 5, }
    cfg_4 = {"environment_num": 10,
             "start_seed": 0, }
    cfg_5 = {"environment_num": 3,
             "start_seed": 3, }
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


if __name__ == '__main__':
    test_map_random_seeding()
