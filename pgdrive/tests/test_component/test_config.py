from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import Config, recursive_equal


def test_config_unchangeable():
    c = Config({"aaa": 100}, unchangeable=True)
    try:
        c['aaa'] = 1000
    except ValueError as e:
        print('Great! ', e)
    assert c['aaa'] == 100


def test_config_sync():
    """
    The config in BaseEngine should be the same as env.config, if BaseEngine exists in process
    """
    try:
        env = PGDriveEnv({"vehicle_config": dict(
            show_lidar=False, show_navi_mark=False
        )})
        env.reset()
        recursive_equal(env.config, env.engine.global_config)
        env.config.update({"vehicle_config": dict(show_lidar=True, show_navi_mark=True)})
        recursive_equal(env.config, env.engine.global_config)
        env.close()
        env.reset()
        recursive_equal(env.config, env.engine.global_config)
        env.engine.global_config.update(
            {"vehicle_config": dict(show_lidar=False, show_navi_mark=False
            )}
        )
        recursive_equal(env.config, env.engine.global_config)
    finally:
        env.close()


if __name__ == '__main__':
    test_config_unchangeable()
