from pgdrive.component.vehicle.base_vehicle import BaseVehicle
from pgdrive.component.vehicle.vehicle_type import SVehicle
import copy
import logging
from collections import namedtuple, deque
from typing import Tuple, Dict
import numpy as np

from pgdrive.component.lane.abs_lane import AbstractLane
from pgdrive.component.map.base_map import BaseMap
from pgdrive.component.road.road import Road
from pgdrive.constants import TARGET_VEHICLES, TRAFFIC_VEHICLES, OBJECT_TO_AGENT, AGENT_TO_OBJECT, ARGOVERSE_AGENT_ID
from pgdrive.engine.engine_utils import get_engine
from pgdrive.manager.base_manager import BaseManager
from pgdrive.utils import norm, merge_dicts, parse_tracking_data
import math

BlockVehicles = namedtuple("block_vehicles", "trigger_road vehicles")


class TrafficMode:
    # Traffic vehicles will be respawn, once they arrive at the destinations
    Respawn = "respawn"

    # Traffic vehicles will be triggered only once
    Trigger = "trigger"

    # Hybrid, some vehicles are triggered once on map and disappear when arriving at destination, others exist all time
    Hybrid = "hybrid"

    # Traffic vehicles are triggered according to real data
    Real = "Real"


class TrafficManager(BaseManager):
    VEHICLE_GAP = 10  # m

    def __init__(self):
        """
        Control the whole traffic flow
        """
        super(TrafficManager, self).__init__()

        self._traffic_vehicles = []
        self.block_triggered_vehicles = []

        # traffic property
        self.mode = self.engine.global_config["traffic_mode"]
        self.random_traffic = self.engine.global_config["random_traffic"]
        self.density = self.engine.global_config["traffic_density"]
        self.respawn_lanes = None

    def reset(self):
        """
        Generate traffic on map, according to the mode and density
        :return: List of Traffic vehicles
        """
        map = self.current_map
        logging.debug("load scene {}".format("Use random traffic" if self.random_traffic else ""))

        # update vehicle list
        self.block_triggered_vehicles = []

        traffic_density = self.density
        if abs(traffic_density) < 1e-2:
            return
        self.respawn_lanes = self.respawn_lanes = self._get_available_respawn_lanes(map)
        if self.mode == TrafficMode.Respawn:
            # add respawn vehicle
            self._create_respawn_vehicles(map, traffic_density)
        elif self.mode == TrafficMode.Trigger or self.mode == TrafficMode.Hybrid:
            self._create_vehicles_once(map, traffic_density)
        elif self.mode == TrafficMode.Real:
            self._create_argoverse_vehicles_once(map)
            self._traffic_vehicles_inited = False
        else:
            raise ValueError("No such mode named {}".format(self.mode))

    def before_step(self):
        """
        All traffic vehicles make driving decision here
        :return: None
        """
        # trigger vehicles
        engine = self.engine
        if self.mode == TrafficMode.Trigger or self.mode == TrafficMode.Hybrid:
            for v in engine.agent_manager.active_objects.values():
                ego_lane_idx = v.lane_index[:-1]
                ego_road = Road(ego_lane_idx[0], ego_lane_idx[1])
                if len(self.block_triggered_vehicles) > 0 and \
                        ego_road == self.block_triggered_vehicles[-1].trigger_road:
                    block_vehicles = self.block_triggered_vehicles.pop()
                    self._traffic_vehicles += block_vehicles.vehicles
        elif self.mode == TrafficMode.Real:
            if not self._traffic_vehicles_inited:
                self._traffic_vehicles = [i.vehicles[0] for i in self.block_triggered_vehicles]
                self._traffic_vehicles_inited = True

        for v in self._traffic_vehicles:
            p = self.engine.get_policy(v.name)
            v.before_step(p.act())
        return dict()

    def after_step(self):
        """
        Update all traffic vehicles' states,
        """
        v_to_remove = []
        for v in self._traffic_vehicles:
            v.after_step()
            if not v.on_lane:
                v_to_remove.append(v)
                # lane = self.respawn_lanes[self.np_random.randint(0, len(self.respawn_lanes))]
                # lane_idx = lane.index
                # long = self.np_random.rand() * lane.length / 2
                # v.update_config({"spawn_lane_index": lane_idx, "spawn_longitude": long})
                # v.reset(self.current_map)
                # self.engine.get_policy(v.id).reset()
        # for v in v_to_remove:
        #     self.clear_objects([v.id])
        #     self._traffic_vehicles.remove(v)
        return dict()

    def before_reset(self) -> None:
        """
        Clear the scene and then reset the scene to empty
        :return: None
        """
        super(TrafficManager, self).before_reset()
        self.density = self.engine.global_config["traffic_density"]
        self.block_triggered_vehicles = []
        self._traffic_vehicles = []

    def get_vehicle_num(self):
        """
        Get the vehicles on road
        :return:
        """
        if self.mode == TrafficMode.Respawn:
            return len(self._traffic_vehicles)
        return sum(len(block_vehicle_set.vehicles) for block_vehicle_set in self.block_triggered_vehicles)

    def get_global_states(self) -> Dict:
        """
        Return all traffic vehicles' states
        :return: States of all vehicles
        """
        states = dict()
        traffic_states = dict()
        for vehicle in self._traffic_vehicles:
            traffic_states[vehicle.index] = vehicle.get_state()

        # collect other vehicles
        if self.mode != TrafficMode.Respawn:
            for v_b in self.block_triggered_vehicles:
                for vehicle in v_b.vehicles:
                    traffic_states[vehicle.index] = vehicle.get_state()
        states[TRAFFIC_VEHICLES] = traffic_states
        active_obj = copy.copy(self.engine.agent_manager._active_objects)
        pending_obj = copy.copy(self.engine.agent_manager._pending_objects)
        dying_obj = copy.copy(self.engine.agent_manager._dying_objects)
        states[TARGET_VEHICLES] = {k: v.get_state() for k, v in active_obj.items()}
        states[TARGET_VEHICLES] = merge_dicts(
            states[TARGET_VEHICLES], {k: v.get_state()
                                      for k, v in pending_obj.items()}, allow_new_keys=True
        )
        states[TARGET_VEHICLES] = merge_dicts(
            states[TARGET_VEHICLES], {k: v_count[0].get_state()
                                      for k, v_count in dying_obj.items()},
            allow_new_keys=True
        )

        states[OBJECT_TO_AGENT] = copy.deepcopy(self.engine.agent_manager._object_to_agent)
        states[AGENT_TO_OBJECT] = copy.deepcopy(self.engine.agent_manager._agent_to_object)
        return states

    def get_global_init_states(self) -> Dict:
        """
        Special handling for first states of traffic vehicles
        :return: States of all vehicles
        """
        vehicles = dict()
        for vehicle in self._traffic_vehicles:
            init_state = vehicle.get_state()
            init_state["index"] = vehicle.index
            init_state["type"] = vehicle.class_name
            init_state["enable_respawn"] = vehicle.enable_respawn
            vehicles[vehicle.index] = init_state

        # collect other vehicles
        if self.mode != TrafficMode.Respawn:
            for v_b in self.block_triggered_vehicles:
                for vehicle in v_b.vehicles:
                    init_state = vehicle.get_state()
                    init_state["type"] = vehicle.class_name
                    init_state["index"] = vehicle.index
                    init_state["enable_respawn"] = vehicle.enable_respawn
                    vehicles[vehicle.index] = init_state
        return vehicles

    def _create_vehicles_on_lane(self, traffic_density: float, lane: AbstractLane, is_respawn_lane):
        """
        Create vehicles on a lane
        :param traffic_density: traffic density according to num of vehicles per meter
        :param lane: Circular lane or Straight lane
        :param is_respawn_lane: Whether vehicles should be respawn on this lane or not
        :return: List of vehicles
        """

        from pgdrive.component.blocks.ramp import InRampOnStraight
        _traffic_vehicles = []
        total_num = int(lane.length / self.VEHICLE_GAP)
        vehicle_longs = [i * self.VEHICLE_GAP for i in range(total_num)]
        self.np_random.shuffle(vehicle_longs)
        for long in vehicle_longs:
            # if self.np_random.rand() > traffic_density and abs(lane.length - InRampOnStraight.RAMP_LEN) > 0.1:
            #     # Do special handling for ramp, and there must be vehicles created there
            #     continue
            vehicle_type = self.random_vehicle_type()
            random_v = self.spawn_object(
                vehicle_type,
                vehicle_config={
                    "spawn_lane_index": lane.index,
                    "spawn_longitude": long,
                    "enable_reverse": False
                }
            )
            from pgdrive.policy.idm_policy import IDMPolicy
            self.engine.add_policy(random_v.id, IDMPolicy(random_v, self.generate_seed()))
            _traffic_vehicles.append(random_v)
        return _traffic_vehicles

    def _propose_vehicle_configs(self, lane: AbstractLane):
        potential_vehicle_configs = []
        total_num = int(lane.length / self.VEHICLE_GAP)
        vehicle_longs = [i * self.VEHICLE_GAP for i in range(total_num)]
        # Only choose given number of vehicles
        for long in vehicle_longs:
            random_vehicle_config = {"spawn_lane_index": lane.index, "spawn_longitude": long, "enable_reverse": False}
            potential_vehicle_configs.append(random_vehicle_config)
        return potential_vehicle_configs

    def _create_respawn_vehicles(self, map: BaseMap, traffic_density: float):
        respawn_lanes = self._get_available_respawn_lanes(map)
        for lane in respawn_lanes:
            self._traffic_vehicles += self._create_vehicles_on_lane(traffic_density, lane, True)

    def _create_argoverse_vehicles_once(self, map: BaseMap) -> None:
        """
        Trigger mode, vehicles will be triggered only once, and disappear when arriving destination
        :param map: Map map.road_network[index]
        :param traffic_density: it can be adjusted each episode
        :return: None
        """
        real_data_config = self.engine.global_config["real_data_config"]
        locate_info = parse_tracking_data(real_data_config["data_path"], real_data_config["log_id"])
        if real_data_config["replay_agent"]:
            del locate_info[ARGOVERSE_AGENT_ID]
        pos_dict = {i:j["init_pos"] for i,j in zip(locate_info.keys(), locate_info.values())}
        
        block = map.blocks[0]
        lanes = block.argo_lanes
        roads = block.block_network.get_roads(direction='positive', lane_num = 1)
        potential_vehicle_configs = []
        for l in lanes:
            if l in self.engine.object_manager.accident_lanes:
                continue
            start = np.max(l.centerline, axis=0)
            end = np.min(l.centerline, axis=0)
            for idx, pos in zip(pos_dict.keys(), pos_dict.values()):
                if start[0] > pos[0] > end[0] and start[1] > pos[1] > end[1]:
                    long, lat = l.local_coordinates(pos)
                    config = {
                        #* 如何控制出生车道?
                        "id": idx,
                        "v_config": {
                            "spawn_lane_index": l.index,
                            "spawn_longitude": long,
                            # "spawn_lateral": lat,
                            "enable_reverse": False,
                        }
                    }
                    potential_vehicle_configs.append(config)
                    pos_dict.pop(idx, None)
                    break

        from pgdrive.policy.replay_policy import ReplayPolicy
        vehicle_type = SVehicle
        for road in roads:
            for config in potential_vehicle_configs:
                v_config = config["v_config"]
                v_start = v_config["spawn_lane_index"][0]
                v_end   = v_config["spawn_lane_index"][1]
                if road.start_node == v_start and road.end_node == v_end:
                    generated_v = self.spawn_object(vehicle_type, vehicle_config=v_config)
                    self.engine.add_policy(generated_v.id, ReplayPolicy(generated_v, locate_info[config["id"]]))
                    block_vehicles = BlockVehicles(trigger_road=road, vehicles=[generated_v])
                    self.block_triggered_vehicles.append(block_vehicles)
                    potential_vehicle_configs.remove(config)
                    break
        self.block_triggered_vehicles.reverse()


    def _create_vehicles_once(self, map: BaseMap, traffic_density: float) -> None:
        """
        Trigger mode, vehicles will be triggered only once, and disappear when arriving destination
        :param map: Map
        :param traffic_density: it can be adjusted each episode
        :return: None
        """
        vehicle_num = 0
        for block in map.blocks[1:]:

            # Propose candidate locations for spawning new vehicles
            trigger_lanes = block.get_intermediate_spawn_lanes()
            potential_vehicle_configs = []
            for lanes in trigger_lanes:
                for l in lanes:
                    if l in self.engine.object_manager.accident_lanes:
                        continue
                    potential_vehicle_configs += self._propose_vehicle_configs(l)

            # How many vehicles should we spawn in this block?
            total_length = sum([lane.length for lanes in trigger_lanes for lane in lanes])
            total_spawn_points = int(math.floor(total_length / self.VEHICLE_GAP))
            total_vehicles = int(math.floor(total_spawn_points * traffic_density))

            # Generate vehicles!
            vehicles_on_block = []
            self.np_random.shuffle(potential_vehicle_configs)
            selected = potential_vehicle_configs[:min(total_vehicles, len(potential_vehicle_configs))]
            # print("We have {} candidates! We are spawning {} vehicles!".format(total_vehicles, len(selected)))

            from pgdrive.policy.idm_policy import IDMPolicy
            for v_config in selected:
                vehicle_type = self.random_vehicle_type()
                random_v = self.spawn_object(vehicle_type, vehicle_config=v_config)
                self.engine.add_policy(random_v.id, IDMPolicy(random_v, self.generate_seed()))
                vehicles_on_block.append(random_v)

            trigger_road = block.pre_block_socket.positive_road
            block_vehicles = BlockVehicles(trigger_road=trigger_road, vehicles=vehicles_on_block)

            self.block_triggered_vehicles.append(block_vehicles)
            vehicle_num += len(vehicles_on_block)
        self.block_triggered_vehicles.reverse()

    def _get_available_respawn_lanes(self, map: BaseMap) -> list:
        """
        Used to find some respawn lanes
        :param map: select spawn lanes from this map
        :return: respawn_lanes
        """
        respawn_lanes = []
        respawn_roads = []
        for block in map.blocks:
            roads = block.get_respawn_roads()
            for road in roads:
                if road in respawn_roads:
                    respawn_roads.remove(road)
                else:
                    respawn_roads.append(road)
        for road in respawn_roads:
            respawn_lanes += road.get_lanes(map.road_network)
        return respawn_lanes

    def random_vehicle_type(self):
        from pgdrive.component.vehicle.vehicle_type import random_vehicle_type
        vehicle_type = random_vehicle_type(self.np_random, [0.2, 0.3, 0.3, 0.2, 0])
        return vehicle_type

    def destroy(self) -> None:
        """
        Destory func, release resource
        :return: None
        """
        self.clear_objects([v.id for v in self._traffic_vehicles])
        self._traffic_vehicles = []
        # current map

        # traffic vehicle list
        self._traffic_vehicles = None
        self.block_triggered_vehicles = None

        # traffic property
        self.mode = None
        self.random_traffic = None
        self.density = None

    def __del__(self):
        logging.debug("{} is destroyed".format(self.__class__.__name__))

    def __repr__(self):
        return self.vehicles.__repr__()

    @property
    def vehicles(self):
        return list(self.engine.get_objects(filter=lambda o: isinstance(o, BaseVehicle)).values())

    @property
    def traffic_vehicles(self):
        return list(self._traffic_vehicles)

    def seed(self, random_seed):
        if not self.random_traffic:
            super(TrafficManager, self).seed(random_seed)

    @property
    def current_map(self):
        return self.engine.map_manager.current_map
