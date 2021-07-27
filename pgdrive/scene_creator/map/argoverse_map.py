from pgdrive.scene_creator.map.base_map import BaseMap
from argoverse.map_representation.map_api import ArgoverseMap


class ArgoMap(BaseMap):
    """
    This class converting the Argoverse to PGDrive and allow the interactive behaviors
    """

    # according to api of get_vector_map_lane_polygons(), the lane width in argoverse dataset is 3.8m
    LANE_WIDTH = 3.8

    # supported city mas
    SUPPORTED_MAPS = ["MIA", "PIT"]

    # load one for all  instance using
    _argoverse_map = ArgoverseMap()

    def __init__(self, map_config, random_seed=0):
        super(ArgoMap, self).__init__(map_config=map_config, random_seed=random_seed)
        assert "city" in map_config, "City name is required when generating argoverse map"
        assert map_config["city"] in self.SUPPORTED_MAPS, "City generation of {} is not supported (We support {} now)". \
            format(map_config["city"], self.SUPPORTED_MAPS)
        self.city = self.config["city"]

    def _generate(self):
        raise NotImplementedError
