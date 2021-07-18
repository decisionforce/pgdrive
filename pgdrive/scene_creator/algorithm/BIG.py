import logging
from typing import Union

from pgdrive.utils.random import RandomEngine
from panda3d.core import NodePath
from pgdrive.scene_creator.blocks.block import Block
from pgdrive.scene_creator.blocks.first_block import FirstBlock
from pgdrive.scene_creator.algorithm.blocks_prob_dist import PGBlock
from pgdrive.scene_creator.road.road_network import RoadNetwork
from pgdrive.engine.core.pg_physics_world import PGPhysicsWorld


class NextStep:
    back = 0
    forward = 1
    search_sibling = 3
    destruct_current = 4


class BigGenerateMethod:
    BLOCK_SEQUENCE = "block_sequence"
    BLOCK_NUM = "block_num"
    SINGLE_BLOCK = "single_block"


class BIG(RandomEngine):
    MAX_TRIAL = 2

    def __init__(
        self,
        lane_num: int,
        lane_width: float,
        global_network: RoadNetwork,
        render_node_path: NodePath,
        pg_physics_world: PGPhysicsWorld,
        block_type_version: str,
        exit_length=50
    ):
        super(BIG, self).__init__()
        self._block_sequence = None
        # Don't change this right now, since we need to make maps identical to old one
        self._lane_num = lane_num
        self._lane_width = lane_width
        self.block_num = None
        self._render_node_path = render_node_path
        self._physics_world = pg_physics_world
        self._global_network = global_network
        self.blocks = []
        self._exit_length = exit_length
        first_block = FirstBlock(
            self._global_network,
            self._lane_width,
            self._lane_num,
            self._render_node_path,
            self._physics_world,
            length=self._exit_length
        )
        self.blocks.append(first_block)
        self.next_step = NextStep.forward
        assert block_type_version in ["v1", "v2"]
        self.block_type_version = block_type_version

    def generate(self, generate_method: str, parameter: Union[str, int]):
        """
        In order to embed it to the show_base loop, we implement BIG in a more complex way
        """
        if generate_method == BigGenerateMethod.BLOCK_NUM:
            assert isinstance(parameter, int), "When generating map by assigning block num, the parameter should be int"
            self.block_num = parameter + 1
        elif generate_method == BigGenerateMethod.BLOCK_SEQUENCE:
            assert isinstance(parameter, str), "When generating map from block sequence, the parameter should be a str"
            self.block_num = len(parameter) + 1
            self._block_sequence = FirstBlock.ID + parameter
        while True:
            if self.big_helper_func():
                break
        return self._global_network

    def big_helper_func(self):
        if len(self.blocks) >= self.block_num and self.next_step == NextStep.forward:
            return True
        if self.next_step == NextStep.forward:
            self._forward()
        elif self.next_step == NextStep.destruct_current:
            self._destruct_current()
        elif self.next_step == NextStep.search_sibling:
            self._search_sibling()
        elif self.next_step == NextStep.back:
            self._go_back()
        return False

    def sample_block(self) -> Block:
        """
        Sample a random block type
        """
        if self._block_sequence is None:
            block_types = PGBlock.all_blocks(self.block_type_version)
            block_probabilities = PGBlock.block_probability(self.block_type_version)
            block_type = self.np_random.choice(block_types, p=block_probabilities)
        else:
            type_id = self._block_sequence[len(self.blocks)]
            block_type = PGBlock.get_block(type_id, self.block_type_version)

        socket = self.np_random.choice(self.blocks[-1].get_socket_indices())
        block = block_type(len(self.blocks), self.blocks[-1].get_socket(socket), self._global_network)
        return block

    def destruct(self, block):
        block.destruct_block(self._physics_world)

    def construct(self, block) -> bool:
        return block.construct_block(self._render_node_path, self._physics_world)

    def _forward(self):
        logging.debug("forward")
        block = self.sample_block()
        self.blocks.append(block)
        success = self.construct(block)
        self.next_step = NextStep.forward if success else NextStep.destruct_current

    def _go_back(self):
        logging.debug("back")
        self.blocks.pop()
        last_block = self.blocks[-1]
        self.destruct(last_block)
        self.next_step = NextStep.search_sibling

    def _search_sibling(self):
        logging.debug("sibling")
        block = self.blocks[-1]
        if block.number_of_sample_trial < self.MAX_TRIAL:
            success = self.construct(block)
            self.next_step = NextStep.forward if success else NextStep.destruct_current
        else:
            self.next_step = NextStep.back

    def _destruct_current(self):
        logging.debug("destruct")
        block = self.blocks[-1]
        self.destruct(block)
        self.next_step = NextStep.search_sibling if block.number_of_sample_trial < self.MAX_TRIAL else NextStep.back

    def __del__(self):
        logging.debug("Destroy Big")
