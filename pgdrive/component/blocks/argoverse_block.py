from pgdrive.component.blocks.base_block import BaseBlock
from pgdrive.component.road.road import Road
from pgdrive.component.road.road_network import RoadNetwork
from typing import Iterable
from pgdrive.component.lane.argoverse_lane import ArgoverseLane


class ArgoverseBlock(BaseBlock):
    def __init__(self, block_index: int, global_network: RoadNetwork, argoverse_lanes: Iterable[ArgoverseLane]):
        """
        No randomization when create argoverse block, Split Argoverse Map to several blocks to boost efficiency
        """
        super(ArgoverseBlock, self).__init__(block_index, global_network, 0)
        self.argo_lanes = argoverse_lanes

    def _sample_topology(self) -> bool:
        for lane in self.argo_lanes:
            self.block_network.add_road(Road(lane.start_node, lane.end_node), [lane])
        return True
