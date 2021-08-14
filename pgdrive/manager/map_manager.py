import json
import logging
import os.path as osp

from pgdrive.component.map.base_map import BaseMap, MapGenerateMethod
from pgdrive.manager.base_manager import BaseManager
from pgdrive.utils import recursive_equal

from pgdrive.component.map.pg_map import PGMap


class MapManager(BaseManager):
    """
    MapManager contains a list of maps
    """
    PRIORITY = 0  # Map update has the most high priority

    def __init__(self):
        super(MapManager, self).__init__()
        self.current_map = None

        # for pgmaps
        start_seed = self.engine.global_config["start_seed"]
        env_num = self.engine.global_config["environment_num"]
        self.restored_pg_map_configs = None
        self.pg_maps = {_seed: None for _seed in range(start_seed, start_seed + env_num)}

    def spawn_object(self, object_class, *args, **kwargs):
        if "random_seed" in kwargs:
            assert kwargs["random_seed"] == self.random_seed, "The random seed assigned is not same as map.seed"
            kwargs.pop("random_seed")
        map = self.engine.spawn_object(object_class, random_seed=self.random_seed, *args, **kwargs)
        self.pg_maps[map.random_seed] = map
        return map

    def load_map(self, map):
        map.attach_to_world()
        self.current_map = map

    def unload_map(self, map):
        map.detach_from_world()
        self.current_map = None

    def read_all_maps_from_json(self, path):
        assert path.endswith(".json")
        assert osp.isfile(path)
        with open(path, "r") as f:
            config_and_data = json.load(f)
        global_config = self.engine.global_config
        start_seed = global_config["start_seed"]
        env_num = global_config["environment_num"]
        if recursive_equal(global_config["map_config"], config_and_data["map_config"]) \
                and set([i for i in range(start_seed, start_seed + env_num)]).issubset(
            set([int(v) for v in config_and_data["map_data"].keys()])):
            self.read_all_maps(config_and_data)
            return True
        else:
            logging.warning(
                "Warning: The pre-generated maps is with config {}, but current environment's map "
                "config is {}.\nWe now fallback to BIG algorithm to generate map online!".format(
                    config_and_data["map_config"], global_config["map_config"]
                )
            )
            global_config["load_map_from_json"] = False  # Don't fall into this function again.
            return False

    def read_all_maps(self, data):
        assert isinstance(data, dict)
        assert set(data.keys()) == {"map_config", "map_data"}
        logging.info(
            "Restoring the maps from pre-generated file! "
            "We have {} maps in the file and restoring {} maps range from {} to {}".format(
                len(data["map_data"]), len(self.pg_maps.keys()), min(self.pg_maps.keys()), max(self.pg_maps.keys())
            )
        )

        maps_collection_config = data["map_config"]
        assert set(self.engine.global_config["map_config"].keys()) == set(maps_collection_config.keys())
        for k in self.engine.global_config["map_config"]:
            assert maps_collection_config[k] == self.engine.global_config["map_config"][k]
        self.restored_pg_map_configs = {}
        # for seed, map_dict in data["map_data"].items():
        for seed, config in data["map_data"].items():
            map_config = {}
            map_config[BaseMap.GENERATE_TYPE] = MapGenerateMethod.PG_MAP_FILE
            map_config[BaseMap.GENERATE_CONFIG] = config
            self.restored_pg_map_configs[seed] = map_config

    def destroy(self):
        self.pg_maps = None
        self.restored_pg_map_configs = None
        super(MapManager, self).destroy()

    def update_map(self, config, current_seed, episode_data: dict = None):
        # TODO(pzh): Remove the config as the input args.
        if episode_data is not None:
            # TODO restore/replay here
            # Since in episode data map data only contains one map, values()[0] is the map_parameters
            map_data = episode_data["map_data"].values()
            assert len(map_data) > 0, "Can not find map info in episode data"
            blocks_info = map_data[0]

            map_config = config["map_config"].copy()
            map_config[BaseMap.GENERATE_TYPE] = MapGenerateMethod.PG_MAP_FILE
            map_config[BaseMap.GENERATE_CONFIG] = blocks_info
            self.spawn_object(PGMap, map_config=map_config)
            return

        # If we choose to load maps from json file.
        if config["load_map_from_json"] and self.current_map is None:
            assert config["_load_map_from_json"]
            self.read_all_maps_from_json(config["_load_map_from_json"])

        # remove map from world before adding
        if self.current_map is not None:
            self.unload_map(self.current_map)

        if self.pg_maps[current_seed] is None:
            if config["load_map_from_json"]:
                map_config = self.restored_pg_map_configs.get(current_seed, None)
                assert map_config is not None
            else:
                map_config = config["map_config"]
                map_config.update({"seed": current_seed})
            print("We are spawning new map. This is the config: ", current_seed, map_config)
            map = self.spawn_object(PGMap, map_config=map_config)
        else:
            print("We are loading map from pg_maps: ", current_seed, len(self.pg_maps))
            map = self.pg_maps[current_seed]
        self.load_map(map)
