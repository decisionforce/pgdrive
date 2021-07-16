from pgdrive.scene_managers.base_manager import BaseManager


class MapManager(BaseManager):
    """
    MapManager contains a list of maps
    """

    def __init__(self):
        super(MapManager, self).__init__()


    def

    def destroy(self):
        del self.maps
        self.maps = {_seed: None for _seed in range(self.start_seed, self.start_seed + self.env_num)}
        del self.current_map
        self.current_map = None
        del self.restored_maps
        self.restored_maps = dict()
        super(MapManager, self).destroy()