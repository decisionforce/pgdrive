from pgdrive import PGDriveEnv


def test_nested_config():
    try:
        config = PGDriveEnv.default_config()
        config.update(dict(
            traffic_density=0.1,
            bbb=0,
            general_penalty=0.0,
        ))
    except KeyError:
        print("Test passed!")
    else:
        raise ValueError("Test failed!")


def test_config_consistency():
    term = "success_reward"
    env = PGDriveEnv({term: -999})
    env.reset()

    values = {
        "outer": env.config[term],
        "vehicle_config": env.config["vehicle_config"][term],
        "multiagent": env.config["target_vehicle_configs"][env.DEFAULT_AGENT][term],
    }
    for key, val in values.items():
        assert -999 == val, (key, val)


if __name__ == '__main__':
    test_nested_config()
    test_config_consistency()
