from pgdrive.envs import PGDriveEnvV2
from pgdrive.utils import setup_logger


def test_reward_cost_done():
    rewards = dict(
        success_reward=1111,
        out_of_road_penalty=2222,
        crash_vehicle_penalty=3333,
        crash_object_penalty=4444,
        out_of_road_cost=5555,
        crash_vehicle_cost=6666,
        crash_object_cost=7777,
    )

    # Success
    # config = rewards.copy()
    # config["map"] = "S"
    # config["traffic_density"] = 0
    # try:
    #     env = PGDriveEnvV2(config=config)
    #     env.reset()
    #     for _ in range(1000):
    #         o, r, d, i = env.step([0, 1])
    #         if d:
    #             break
    #     assert i["arrive_dest"]
    #     assert i["cost"] == 0
    #     assert r == rewards["success_reward"]
    # finally:
    #     if "env" in locals():
    #         env.close()

    # Out of road
    # config = rewards.copy()
    # config["map"] = "S"
    # config["traffic_density"] = 0
    # try:
    #     env = PGDriveEnvV2(config=config)
    #     env.reset()
    #     for _ in range(1000):
    #         o, r, d, i = env.step([1, 1])
    #         if d:
    #             break
    #     assert i["out_of_road"]
    #     assert i["cost"] == rewards["out_of_road_cost"]
    #     assert r == -rewards["out_of_road_penalty"]
    # finally:
    #     if "env" in locals():
    #         env.close()

    # Crash vehicle
    config = rewards.copy()
    config["map"] = "SSS"
    config["traffic_density"] = 20
    try:
        env = PGDriveEnvV2(config=config)
        env.reset()
        epr = 0
        for _ in range(1000):
            o, r, d, i = env.step([0, 1])
            epr += r
            print("R: {}, Accu R: {}".format(r, epr))
            if d:
                epr = 0
                break
        assert i["crash"]
        assert i["crash_vehicle"]
        assert i["cost"] == rewards["crash_vehicle_cost"]
        assert r == -rewards["crash_vehicle_penalty"]
    finally:
        if "env" in locals():
            env.close()

    # # Crash object
    # config = rewards.copy()
    # config["map"] = "SSS"
    # config["traffic_density"] = 0
    # config['accident_prob'] = 1.0
    # # config["use_render"] = True
    # # config["fast"] = True
    # config["pg_world_config"] = {"_debug_crash_object": True}
    # try:
    #     env = PGDriveEnvV2(config=config)
    #     env.reset()
    #     for _ in range(1000):
    #         o, r, d, i = env.step([0, 1])
    #         if d:
    #             break
    #     assert i["crash"]
    #     assert i["crash_object"]
    #     assert i["cost"] == rewards["crash_object_cost"]
    #     assert r == -rewards["crash_object_penalty"]
    # finally:
    #     if "env" in locals():
    #         env.close()


if __name__ == '__main__':
    setup_logger(True)
    test_reward_cost_done()
