from pgdrive.component.vehicle.base_vehicle import BaseVehicle
import copy
import logging
from collections import namedtuple, deque
from typing import Tuple, Dict

from pgdrive.component.lane.abs_lane import AbstractLane
from pgdrive.component.map.base_map import BaseMap
from pgdrive.component.road.road import Road
from pgdrive.constants import TARGET_VEHICLES, TRAFFIC_VEHICLES, OBJECT_TO_AGENT, AGENT_TO_OBJECT
from pgdrive.engine.engine_utils import get_engine
from pgdrive.manager.base_manager import BaseManager
from pgdrive.utils import norm, merge_dicts

BlockVehicles = namedtuple("block_vehicles", "trigger_road vehicles")


class TrafficMode:
    # Traffic vehicles will be respawn, once they arrive at the destinations
    Respawn = "respawn"

    # Traffic vehicles will be triggered only once
    Trigger = "trigger"

    # Hybrid, some vehicles are triggered once on map and disappear when arriving at destination, others exist all time
    Hybrid = "hybrid"


class TrafficManager(BaseManager):
    VEHICLE_GAP = 10  # m

    def __init__(self):
        """
        Control the whole traffic flow
        """
        super(TrafficManager, self).__init__()

        self._traffic_vehicles = []
        self.block_triggered_vehicles = None

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
        logging.debug("load scene {}, {}".format(map.random_seed, "Use random traffic" if self.random_traffic else ""))

        # update vehicle list
        self.block_triggered_vehicles = [] if self.mode != TrafficMode.Respawn else None

        traffic_density = self.density
        if abs(traffic_density - 0.0) < 1e-2:
            return
        self.respawn_lanes = self.respawn_lanes = self._get_available_respawn_lanes(map)
        if self.mode == TrafficMode.Respawn:
            # add respawn vehicle
            self._create_respawn_vehicles(map, traffic_density)
        elif self.mode == TrafficMode.Trigger or self.mode == TrafficMode.Respawn:
            self._create_vehicles_once(map, traffic_density)
        else:
            raise ValueError("No such mode named {}".format(self.mode))

    def before_step(self):
        """
        All traffic vehicles make driving decision here
        :return: None
        """
        # trigger vehicles
        engine = self.engine
        if self.mode != TrafficMode.Respawn:
            for v in engine.agent_manager.active_objects.values():
                ego_lane_idx = v.lane_index[:-1]
                ego_road = Road(ego_lane_idx[0], ego_lane_idx[1])
                if len(self.block_triggered_vehicles) > 0 and \
                        ego_road == self.block_triggered_vehicles[-1].trigger_road:
                    block_vehicles = self.block_triggered_vehicles.pop()
                    self._traffic_vehicles += block_vehicles.vehicles
        for v in self._traffic_vehicles:
            p = self.engine.get_policy(v.name)
            v.before_step(p.act())
        return dict()

    def after_step(self):
        """
        Update all traffic vehicles' states,
        """
        for v in self._traffic_vehicles:
            if v.arrive_destination or v.crash_vehicle or v.crash_object:
                lane = self.respawn_lanes[self.np_random.randint(0, len(self.respawn_lanes))]
                lane_idx = lane.index
                long = self.np_random.rand() * lane.length / 2
                v.update_config({"spawn_lane_index": lane_idx, "spawn_longitude": long})
                v.reset(self.current_map)
                self.engine.get_policy(v.id).reset()
            else:
                v.after_step()

    def before_reset(self) -> None:
        """
        Clear the scene and then reset the scene to empty
        :return: None
        """
        self.density = self.engine.global_config["traffic_density"]
        self.engine.clear_objects(filter=[v.id for v in self._traffic_vehicles])

        self.block_triggered_vehicles = [] if self.mode != TrafficMode.Respawn else None
        self._traffic_vehicles = deque()  # it is used to step all vehicles on scene

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

    def spawn_object(self, vehicle_type, lane: AbstractLane, long: float, *args, **kwargs):
        """
        Create one vehicle on lane and a specific place
        :param vehicle_type: TrafficVehicle type (s,m,l,xl)
        :param lane: Straight Lane or Circular Lane
        :param long: longitude position on lane
        :return: TrafficVehicle
        """
        random_v = self.engine.spawn_object(vehicle_type, vehicle_config={"spawn_lane_index": lane.index,
                                                                          "spawn_longitude": long})
        random_v.reset(self.current_map)
        from pgdrive.policy.idm_policy import IDMPolicy
        self.engine.add_policy(random_v.id, IDMPolicy(random_v, self.randint()))
        return random_v

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
            if self.np_random.rand() > traffic_density and abs(lane.length - InRampOnStraight.RAMP_LEN) > 0.1:
                # Do special handling for ramp, and there must be vehicles created there
                continue
            vehicle_type = self.random_vehicle_type()
            _traffic_vehicles.append(self.spawn_object(vehicle_type, lane, long, is_respawn_lane))
        return _traffic_vehicles

    def _create_respawn_vehicles(self, map: BaseMap, traffic_density: float):
        respawn_lanes = self._get_available_respawn_lanes(map)
        for lane in respawn_lanes:
            self._traffic_vehicles += self._create_vehicles_on_lane(traffic_density, lane, True)

    def _create_vehicles_once(self, map: BaseMap, traffic_density: float) -> None:
        """
        Trigger mode, vehicles will be triggered only once, and disappear when arriving destination
        :param map: Map
        :param traffic_density: it can be adjusted each episode
        :return: None
        """
        vehicle_num = 0
        for block in map.blocks[1:]:
            if block.PROHIBIT_TRAFFIC_GENERATION:
                continue
            vehicles_on_block = []
            trigger_road = block.pre_block_socket.positive_road

            # trigger lanes is a two dimension array [[]], the first dim represent road consisting of lanes.
            trigger_lanes = block.block_network.get_positive_lanes()
            respawn_lanes = block.get_respawn_lanes()
            for lanes in respawn_lanes:
                if lanes not in trigger_lanes:
                    trigger_lanes.append(lanes)
            self.np_random.shuffle(trigger_lanes)
            for lanes in trigger_lanes:
                num = min(int(len(lanes) * traffic_density) + 1, len(lanes))
                lanes = self.np_random.choice(lanes, num, replace=False) if len(lanes) != 1 else lanes
                for l in lanes:
                    vehicles_on_block += self._create_vehicles_on_lane(traffic_density, l, False)
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
        from pgdrive.component.vehicle.traffic_vehicle_type import vehicle_type
        vehicle_type = vehicle_type[self.np_random.choice(list(vehicle_type.keys()), p=[0., 0., 0., 1.0])]
        return vehicle_type

    def destroy(self) -> None:
        """
        Destory func, release resource
        :return: None
        """
        self.clear_objects()
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
