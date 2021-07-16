from pgdrive.scene_managers.base_manager import BaseManager


class MapManager(BaseManager):
    """
    MapManager contains a list of maps
    """
    def __init__(self):
        super(MapManager, self).__init__()

    def destroy(self):
        super(MapManager, self).destroy()
