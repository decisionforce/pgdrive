
class MapManager:
    """
    MapManager contains a list of maps
    """

    def __init__(self):
        # map setting
        self.start_seed = self.config["start_seed"]
        self.env_num = self.config["environment_num"]
        self.maps = {_seed: None for _seed in range(self.start_seed, self.start_seed + self.env_num)}
        self.current_seed = self.start_seed
        self.current_map = None
        self.restored_maps = dict()

    def destroy(self):
        del self.maps
        self.maps = {_seed: None for _seed in range(self.start_seed, self.start_seed + self.env_num)}
        del self.current_map
        self.current_map = None
        del self.restored_maps
        self.restored_maps = dict()