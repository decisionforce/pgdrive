import logging
from pgdrive.scene_creator.road.road import Road
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Mapping, Tuple, Union, cast

from argoverse.data_loading.vector_map_loader import Node, append_additional_key_value_pair, \
    append_unique_key_value_pair, convert_node_id_list_to_xy, extract_node_waypt, get_lane_identifier, str_to_bool, \
    extract_node_from_ET_element
from argoverse.map_representation.map_api import PITTSBURGH_ID, MIAMI_ID, ROOT

from pgdrive.scene_creator.lane.argoverse_lane import ArgoverseLane

logger = logging.getLogger(__name__)

_PathLike = Union[str, "os.PathLike[str]"]

from pgdrive.scene_creator.map.base_map import BaseMap


class ArgoverseMap(BaseMap):
    """
    This class converting the Argoverse to PGDrive and allow the interactive behaviors
    """

    # according to api of get_vector_map_lane_polygons(), the lane width in argoverse dataset is 3.8m
    LANE_WIDTH = 3.8

    # supported city mas
    SUPPORTED_MAPS = ["MIA", "PIT"]

    def __init__(self, map_config, random_seed=0):
        map_config[self.SEED]=random_seed
        super(ArgoverseMap, self).__init__(map_config=map_config, random_seed=random_seed)
        assert "city" in map_config, "City name is required when generating argoverse map"
        assert map_config["city"] in self.SUPPORTED_MAPS, "City generation of {} is not supported (We support {} now)". \
            format(map_config["city"], self.SUPPORTED_MAPS)
        self.city = self.config["city"]

    def _generate(self):
        """
        Modified from argoverse-map api
        """
        city_name = self.config["city"]
        city_id = MIAMI_ID if city_name == "MIA" else PITTSBURGH_ID
        xml_fpath = Path(ROOT).resolve() / f"pruned_argoverse_{city_name}_{city_id}_vector_map.xml"
        tree = ET.parse(os.fspath(xml_fpath))
        root = tree.getroot()
        logger.info(f"Loaded root: {root.tag}")

        all_graph_nodes = {}
        lane_objs = {}
        # all children are either Nodes or Ways
        for child in root:
            if child.tag == "node":
                node_obj = extract_node_from_ET_element(child)
                all_graph_nodes[node_obj.id] = node_obj
            elif child.tag == "way":
                lane_obj, lane_id = self.extract_lane_segment_from_ET_element(child, all_graph_nodes)
                lane_objs[lane_id] = lane_obj
            else:
                logger.error("Unknown XML item encountered.")
                raise ValueError("Unknown XML item encountered.")
        self._construct_road_network(lane_objs)

    def _construct_road_network(self, lane_dict: dict):
        for lane in lane_dict.values():
            self.road_network.add_road(Road(lane.start_node, lane.end_node), [lane])
        print("finish")

    @staticmethod
    def extract_lane_segment_from_ET_element(child: ET.Element, all_graph_nodes: Mapping[int, Node]
                                             ) -> Tuple[ArgoverseLane, int]:
        """
        Modified from Argoverse map-api
        """
        lane_dictionary: Dict[str, Any] = {}
        lane_id = get_lane_identifier(child)
        node_id_list: List[int] = []
        for element in child:
            # The cast on the next line is the result of a typeshed bug.  This really is a List and not a ItemsView.
            way_field = cast(List[Tuple[str, str]], list(element.items()))
            field_name = way_field[0][0]
            if field_name == "k":
                key = way_field[0][1]
                if key in {"predecessor", "successor"}:
                    append_additional_key_value_pair(lane_dictionary, way_field)
                else:
                    append_unique_key_value_pair(lane_dictionary, way_field)
            else:
                node_id_list.append(extract_node_waypt(way_field))

        lane_dictionary["centerline"] = convert_node_id_list_to_xy(node_id_list, all_graph_nodes)
        predecessors = lane_dictionary.get("predecessor", None)
        successors = lane_dictionary.get("successor", None)
        has_traffic_control = str_to_bool(lane_dictionary["has_traffic_control"])
        is_intersection = str_to_bool(lane_dictionary["is_intersection"])
        lnid = lane_dictionary["l_neighbor_id"]
        rnid = lane_dictionary["r_neighbor_id"]
        l_neighbor_id = None if lnid == "None" else int(lnid)
        r_neighbor_id = None if rnid == "None" else int(rnid)
        lane_segment = ArgoverseLane(
            str(node_id_list[0]),
            str(node_id_list[-1]),
            lane_id,
            has_traffic_control,
            lane_dictionary["turn_direction"],
            is_intersection,
            l_neighbor_id,
            r_neighbor_id,
            predecessors,
            successors,
            lane_dictionary["centerline"],
        )
        return lane_segment, lane_id


if __name__ == "__main__":
    map = ArgoverseMap({"city": "MIA", "draw_map_resolution":1024})
