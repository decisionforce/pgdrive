import copy
from pgdrive.component.vehicle.base_vehicle import BaseVehicle
from math import floor
from typing import Union, List, Dict
import numpy as np
from panda3d.bullet import BulletBoxShape, BulletGhostNode
from panda3d.core import Vec3

from pgdrive.component.blocks.first_block import FirstPGBlock
from pgdrive.component.lane.straight_lane import StraightLane
from pgdrive.constants import CollisionGroup
from pgdrive.engine.engine_utils import get_engine
from pgdrive.utils import Config, get_np_random
from pgdrive.utils.coordinates_shift import panda_position, panda_heading
from pgdrive.utils.scene_utils import rect_region_detection
from pgdrive.manager.base_manager import BaseManager


class SpawnManager(BaseManager):
    """
    This class maintain a list of possible spawn places for MARL respawn
    """
    REGION_DETECT_HEIGHT = 10
    RESPAWN_REGION_LONGITUDE = 8.
    RESPAWN_REGION_LATERAL = 3.
    MAX_VEHICLE_LENGTH = BaseVehicle.MAX_LENGTH
    MAX_VEHICLE_WIDTH = BaseVehicle.MAX_WIDTH

    def __init__(self):
        self.num_agents = self.engine.global_config["num_agents"]
        self.exit_length = (self.engine.global_config["map_config"]["exit_length"] - FirstPGBlock.ENTRANCE_LENGTH)
        assert self.exit_length >= self.RESPAWN_REGION_LONGITUDE, (
            "The exist length {} should greater than minimal longitude interval {}.".format(
                self.exit_length, self.RESPAWN_REGION_LONGITUDE
            )
        )
        self.lane_num = self.engine.global_config["map_config"]["lane_num"]
        self.spawn_roads = []
        self.safe_spawn_places = {}
        self.need_update_spawn_places = True
        self.initialized = False
        self.spawn_places_used = []

        target_vehicle_configs = self.engine.global_config["target_vehicle_configs"],
        self.target_vehicle_configs:Union[List, Dict] = target_vehicle_configs
        self.custom_target_vehicle_config = True if target_vehicle_configs is not None and len(
            target_vehicle_configs) > 0 else False
        if self.custom_target_vehicle_config:
            assert len(self.target_vehicle_configs) == self.engine.global_config["num_agents"]

        if self.num_agents is None:
            assert not self.target_vehicle_configs, (
                "You should now specify config if requiring infinite number of vehicles.")

        spawn_roads = self.engine.global_config["spawn_roads"]
        if self.target_vehicle_configs:
            target_vehicle_configs, safe_spawn_places = self._update_spawn_roads_with_configs(spawn_roads)
        else:
            target_vehicle_configs, safe_spawn_places = self._update_spawn_roads_randomly(spawn_roads)
        self.target_vehicle_configs = target_vehicle_configs
        self.safe_spawn_places = {place["identifier"]: place for place in safe_spawn_places}
        self.spawn_roads = spawn_roads
        self.need_update_spawn_places = True

    def _update_spawn_roads_with_configs(self, spawn_roads=None):
        assert self.num_agents <= len(self.target_vehicle_configs), (
            "Too many agents! We only accept {} agents, which is specified by the number of configs in "
            "target_vehicle_configs, but you have {} agents! "
            "You should require less agent or not to specify the target_vehicle_configs!".format(
                len(self.target_vehicle_configs), self.num_agents
            )
        )
        target_vehicle_configs = []
        safe_spawn_places = []
        for v_id, v_config in self.target_vehicle_configs.items():
            lane_tuple = v_config["spawn_lane_index"]
            target_vehicle_configs.append(
                Config(
                    dict(identifier="|".join((str(s) for s in lane_tuple)), config=v_config),
                    unchangeable=True
                )
            )
            safe_spawn_places.append(target_vehicle_configs[-1].copy())
        return target_vehicle_configs, safe_spawn_places

    def reset(self):
        if not self.initialized:
            super(SpawnManager, self).__init__()
            self.initialized = True

    def _update_spawn_roads_randomly(self, spawn_roads):
        assert not self.custom_target_vehicle_config, "This will overwrite your custom target vehicle config"
        assert len(spawn_roads) > 0
        interval = self.RESPAWN_REGION_LONGITUDE
        num_slots = int(floor(self.exit_length / interval))
        assert num_slots > 0, "The exist length {} should greater than minimal longitude interval {}.".format(
            self.exit_length, interval
        )
        interval = self.exit_length / num_slots
        self._longitude_spawn_interval = interval
        if self.num_agents is not None:
            assert self.num_agents > 0 or self.num_agents == -1
            assert self.num_agents <= self.lane_num * len(spawn_roads) * num_slots, (
                "Too many agents! We only accepet {} agents, but you have {} agents!".format(
                    self.lane_num * len(spawn_roads) * num_slots, self.num_agents
                )
            )

        # We can spawn agents in the middle of road at the initial time, but when some vehicles need to be respawn,
        # then we have to set it to the farthest places to ensure safety (otherwise the new vehicles may suddenly
        # appear at the middle of the road!)
        target_vehicle_configs = []
        safe_spawn_places = []
        for i, road in enumerate(spawn_roads):
            for lane_idx in range(self.lane_num):
                for j in range(num_slots):
                    long = 1 / 2 * self.RESPAWN_REGION_LONGITUDE + j * self.RESPAWN_REGION_LONGITUDE
                    lane_tuple = road.lane_index(lane_idx)  # like (>>>, 1C0_0_, 1) and so on.
                    target_vehicle_configs.append(
                        Config(
                            dict(
                                identifier="|".join((str(s) for s in lane_tuple + (j, ))),
                                config={
                                    "spawn_lane_index": lane_tuple,
                                    "spawn_longitude": long,
                                    "spawn_lateral": 0
                                },
                            ),
                            unchangeable=True
                        )
                    )  # lock the spawn positions
                    if j == 0:
                        safe_spawn_places.append(target_vehicle_configs[-1].copy())
        return target_vehicle_configs, safe_spawn_places

    def get_target_vehicle_configs(self, seed=None):
        # don't overwrite
        if self.custom_target_vehicle_config:
            ret = {}
            for bp in self.target_vehicle_configs:
                v_config = bp["config"]
                ret[bp["force_agent_name"]] = v_config
            return copy.deepcopy(ret)

        num_agents = self.num_agents if self.num_agents is not None else len(self.target_vehicle_configs)
        assert len(self.target_vehicle_configs) > 0

        if num_agents == -1:  # Infinite number of agents
            target_agents = list(range(len(self.target_vehicle_configs)))
        else:
            target_agents = get_np_random(seed).choice(
                [i for i in range(len(self.target_vehicle_configs))], num_agents, replace=False
            )

        # for rllib compatibility
        ret = {}
        if len(target_agents) > 1:
            for real_idx, idx in enumerate(target_agents):
                v_config = self.target_vehicle_configs[idx]["config"]
                v_config = self._randomize_position_in_slot(v_config)
                ret["agent{}".format(real_idx)] = v_config
        else:
            ret["agent0"] = self._randomize_position_in_slot(self.target_vehicle_configs[0]["config"])
        return copy.deepcopy(ret)

    def step(self):
        self.spawn_places_used = []

    def get_available_respawn_places(self, map, randomize=False):
        """
        In each episode, we allow the vehicles to respawn at the start of road, randomize will give vehicles a random
        position in the respawn region
        """
        engine = get_engine()
        ret = {}
        for bid, bp in self.safe_spawn_places.items():
            if bid in self.spawn_places_used:
                continue

            # save time calculate once
            if not bp.get("spawn_point_position", False):
                lane = map.road_network.get_lane(bp["config"]["spawn_lane_index"])
                assert isinstance(lane, StraightLane), "Now we don't support respawn on circular lane"
                long = self.RESPAWN_REGION_LONGITUDE / 2
                spawn_point_position = lane.position(longitudinal=long, lateral=0)
                bp.force_update(
                    {
                        "spawn_point_heading": np.rad2deg(lane.heading_at(long)),
                        "spawn_point_position": (spawn_point_position[0], spawn_point_position[1])
                    }
                )

            spawn_point_position = bp["spawn_point_position"]
            lane_heading = bp["spawn_point_heading"]
            result = rect_region_detection(
                engine, spawn_point_position, lane_heading, self.RESPAWN_REGION_LONGITUDE, self.RESPAWN_REGION_LATERAL,
                CollisionGroup.Vehicle
            )
            if (engine.global_config["debug"] or engine.global_config["debug_physics_world"]) \
                    and bp.get("need_debug", True):
                shape = BulletBoxShape(Vec3(self.RESPAWN_REGION_LONGITUDE / 2, self.RESPAWN_REGION_LATERAL / 2, 1))
                vis_body = engine.render.attach_new_node(BulletGhostNode("debug"))
                vis_body.node().addShape(shape)
                vis_body.setH(panda_heading(lane_heading))
                vis_body.setPos(panda_position(spawn_point_position, z=2))
                engine.physics_world.dynamic_world.attach(vis_body.node())
                vis_body.node().setIntoCollideMask(CollisionGroup.AllOff)
                bp.force_set("need_debug", False)

            if not result.hasHit():
                new_bp = copy.deepcopy(bp).get_dict()
                if randomize:
                    new_bp["config"] = self._randomize_position_in_slot(new_bp["config"])
                ret[bid] = new_bp
                self.spawn_places_used.append(bid)
        return ret

    def _randomize_position_in_slot(self, target_vehicle_config):
        vehicle_config = copy.deepcopy(target_vehicle_config)
        long = self.RESPAWN_REGION_LONGITUDE - self.MAX_VEHICLE_LENGTH
        lat = self.RESPAWN_REGION_LATERAL - self.MAX_VEHICLE_WIDTH
        vehicle_config["spawn_longitude"] += get_np_random(self._seed).uniform(-long / 2, long / 2)
        vehicle_config["spawn_lateral"] += get_np_random(self._seed).uniform(-lat / 2, lat / 2)
        return vehicle_config
