import os

from pg_drive.envs.generalization_racing import GeneralizationRacing
from pg_drive.scene_creator.map import Map, MapGenerateMethod

from pg_drive.utils import setup_logger
import json

setup_logger(debug=True)



def recursive_assert(data1, data2):
    if isinstance(data1, dict):
        assert isinstance(data2, dict)
        assert set(data1.keys()) == set(data2.keys()), (data1.keys(), data2.keys())
        for k in data1:
            recursive_assert(data1[k], data2[k])

    elif isinstance(data1, list):
        assert len(data1) == len(data2)
        for i in range(len(data1)):
            recursive_assert(data1[i], data2[i])

    else:
        assert data1 == data2

if __name__ == "__main__":
    env = GeneralizationRacing({
        "environment_num": 10,
    })
    data = env.dump_all_maps()
    with open("test_10maps.json", "w") as f:
        json.dump(data, f)

    with open("test_10maps.json", "r") as f:
        restored_data = json.load(f)

    env = GeneralizationRacing({
        "environment_num": 10,
        "_load_map_from_json": True
    })
    env.load_all_maps(restored_data)

    for i in range(10):
        m = env.maps[i].save_map()
        recursive_assert(m, data["map_data"][i])
