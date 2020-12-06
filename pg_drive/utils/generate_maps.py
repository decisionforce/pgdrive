import json
import os.path as osp

from pg_drive import GeneralizationRacing

root = osp.dirname(osp.dirname(osp.dirname(osp.abspath(__file__))))
assert_path = osp.join(root, "asset", "maps")

environment_set_dict = {
    "PGDrive-test-v0": {"start_seed": 0, "environment_num": 50},
    "PGDrive-validation-v0": {"start_seed": 200, "environment_num": 800},
    "PGDrive-v0": {"start_seed": 1000, "environment_num": 100},
    "PGDrive-10envs-v0": {"start_seed": 1000, "environment_num": 10},
    "PGDrive-1000envs-v0": {"start_seed": 1000, "environment_num": 1000},
    "PGDrive-debug-v0": {"start_seed": 0, "environment_num": 1,
                         "map_config": {"type": "block_sequence", "config": "CC"}},
}

if __name__ == '__main__':
    print("Root path is {}. Asset path is {}.".format(root, assert_path))
    # Generate the first round
    for env_name, env_config in environment_set_dict.items():
        env = GeneralizationRacing(env_config)
        data = env.dump_all_maps()
        file_path = osp.join(assert_path, "{}.json".format(env_name))
        with open(file_path, "w") as f:
            json.dump(data, f)
        env.close()
        print("Finish environment: ", env_name)

    # # Generate the second round
    # for env_name, env_config in environment_set_dict.items():
    #     env = GeneralizationRacing(env_config)
    #     data = env.dump_all_maps()
    #     file_path = osp.join(assert_path, "{}-quanyi.json".format(env_name))
    #     with open(file_path, "w") as f:
    #         json.dump(data, f)
    #     env.close()
    #     print("Finish environment: ", env_name)
    #
    # for env_name, env_config in environment_set_dict.items():
    #     with open(osp.join(assert_path, "{}.json".format(env_name)), "r") as f:
    #         data_zhenghao = json.load(f)
    #     with open(osp.join(assert_path, "{}-quanyi.json".format(env_name)), "r") as f:
    #         data_quanyi = json.load(f)
    #     recursive_assert(data_zhenghao, data_quanyi)
