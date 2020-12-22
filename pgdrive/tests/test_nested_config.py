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

    # we don't do a recursive check now

    # try:
    #     config = PGDriveEnv.default_config()
    #     config.update(dict(
    #         traffic_density=0.1,
    #         map_config=dict(bbb=0),
    #         general_penalty=0.0,
    #     ))
    # except KeyError:
    #     print("Test passed!")
    # else:
    #     raise ValueError("Test failed!")

    # config = PGDriveEnv.default_config()
    # config["pg_world_config"] = {
    #     "use_render": False,
    # }
    # # Should OK
    # config.update(dict(pg_world_config=dict(use_render=True), ))
    # # Should not OK
    # try:
    #     config.update(dict(pg_world_config=dict(bbb=0), ))
    # except KeyError:
    #     print("Test passed!")
    # else:
    #     raise ValueError("Test failed!")


if __name__ == '__main__':
    test_nested_config()
