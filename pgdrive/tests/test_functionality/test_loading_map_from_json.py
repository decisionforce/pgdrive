import json

from pgdrive import PGDriveEnv, PGDriveEnvV2
from pgdrive.utils import recursive_equal
from pgdrive.utils.generate_maps import generate_maps


def test_v1_v2_alignment():
    # Generate the second round
    # for seed in [0, 1, 2, 100, 200, 300, 9999]:
    for seed in [0, 1, 2, 99]:
        env_config = {
            "start_seed": seed,
            "environment_num": 1
        }
        generate_maps(PGDriveEnv, env_config.copy(), json_file_path="seed{}_v1.json".format(seed))
        generate_maps(PGDriveEnvV2, env_config.copy(), json_file_path="seed{}_v2.json".format(seed))
        with open("seed{}_v1.json".format(seed), "r") as f:
            saved_v1 = json.load(f)
        with open("seed{}_v2.json".format(seed), "r") as f:
            saved_v2 = json.load(f)

        e = PGDriveEnv(env_config.copy())
        e.reset()
        assert e.engine.global_config["load_map_from_json"] is True, (
            "If this assertion fail, it means you are not using the predefined maps. Please check the read_all_"
            "maps_from_json function in map_manager.py"
        )
        map_data_realtime = e.current_map.save_map()
        map_data_in_json = saved_v1["map_data"][str(seed)]
        e.close()

        recursive_equal(saved_v1, saved_v2, True)
        recursive_equal(map_data_in_json, map_data_realtime, True)

    print(saved_v1)


if __name__ == '__main__':
    test_v1_v2_alignment()
