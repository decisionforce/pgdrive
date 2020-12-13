from typing import List, Tuple

LaneIndex = Tuple[str, str, int]
Route = List[LaneIndex]


class Road:
    """
    Road is a bunch of lanes connecting two nodes, one start and the other end
    """
    NEGATIVE_DIR = "-"

    def __init__(self, start_node: str, end_node: str):
        self.start_node = start_node
        self.end_node = end_node

    def get_lanes(self, road_network):
        from pgdrive.scene_creator.basic_utils import get_lanes_on_road
        return get_lanes_on_road(self, road_network)

    def __neg__(self):
        sub_index = self.end_node.find(Road.NEGATIVE_DIR)
        if sub_index == -1:
            return Road(Road.NEGATIVE_DIR + self.end_node, Road.NEGATIVE_DIR + self.start_node)
        else:
            return Road(self.end_node[sub_index + 1:], self.start_node[sub_index + 1:])

    def is_negative_road(self):
        return False if self.end_node.find(Road.NEGATIVE_DIR) == -1 else True

    def is_valid_road(self):
        from pgdrive.scene_creator.basic_utils import Decoration
        return False if self.start_node == Decoration.start and self.end_node == Decoration.end else True

    def __eq__(self, other):
        return True if self.start_node == other.start_node and self.end_node == other.end_node else False

    def __repr__(self):
        return "Road from {} to {}".format(self.start_node, self.end_node)
