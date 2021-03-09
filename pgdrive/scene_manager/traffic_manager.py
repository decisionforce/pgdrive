import logging
from collections import namedtuple, deque
from typing import Tuple, Dict, List

from pgdrive.scene_creator.lane.abs_lane import AbstractLane
from pgdrive.scene_creator.map import Map
from pgdrive.scene_creator.road.road import Road
from pgdrive.utils import norm, RandomEngine
from pgdrive.world.pg_world import PGWorld

BlockVehicles = namedtuple("block_vehicles", "trigger_road vehicles")


class TrafficMode:
    # Traffic vehicles will be reborn, once they arrive at the destinations
    Reborn = "reborn"

    # Traffic vehicles will be triggered only once
    Trigger = "trigger"

    # Hybrid, some vehicles are triggered once on map and disappear when arriving at destination, others exist all time
    Hybrid = "hybrid"


class TrafficManager(RandomEngine):
    VEHICLE_GAP = 10  # m

    def __init__(self, traffic_mode: TrafficMode, random_traffic: bool):
        """
        Control the whole traffic flow
        :param traffic_mode: Reborn mode or Trigger mode
        :param random_traffic: the distribution of vehicles will be different in different episdoes
        """
        # current map TODO maintain one map in scene_mgr
        self.map = None

        # traffic vehicle list
        self.ego_vehicle = None
        self.controllable_vehicles = None
        self.vehicles = None
        self.traffic_vehicles = None
        self.block_triggered_vehicles = None
        self._spawned_vehicles = []  # auto-destroy

        # traffic property
        self.mode = traffic_mode
        self.random_traffic = random_traffic
        self.density = 0
        self.reborn_lanes = None

        # control randomness of traffic
        super(TrafficManager, self).__init__()

    def generate(self, pg_world: PGWorld):
        """
        Generate traffic on map, according to the mode and density
        :param pg_world: World
        :return: List of Traffic vehicles
        """
        traffic_density = self.density
        if abs(traffic_density - 0.0) < 1e-2:
            return
        map = self.map
        self.reborn_lanes = None
        if self.mode == TrafficMode.Reborn:
            # add reborn vehicle
            self._create_reborn_vehicles(pg_world, map, traffic_density)
        elif self.mode == TrafficMode.Trigger:
            self._create_vehicles_once(pg_world, map, traffic_density)
        elif self.mode == TrafficMode.Hybrid:
            # vehicles will be reborn after arriving destination
            self.reborn_lanes = self._get_available_reborn_lanes(map)
            self._create_vehicles_once(pg_world, map, traffic_density)
        else:
            raise ValueError("No such mode named {}".format(self.mode))
        logging.debug("Init {} Traffic Vehicles".format(len(self._spawned_vehicles)))

    def prepare_step(self, scene_mgr):
        """
        All traffic vehicles make driving decision here
        :param scene_mgr: access other elements in scene
        :param pg_world: World
        :return: None
        """
        if self.mode != TrafficMode.Reborn:
            ego_lane_idx = self.ego_vehicle.lane_index[:-1]
            ego_road = Road(ego_lane_idx[0], ego_lane_idx[1])
            if len(self.block_triggered_vehicles) > 0 and ego_road == self.block_triggered_vehicles[-1].trigger_road:
                block_vehicles = self.block_triggered_vehicles.pop()
                self.traffic_vehicles += block_vehicles.vehicles
        for v in self.traffic_vehicles:
            v.prepare_step(scene_mgr=scene_mgr)

    def step(self, dt: float):
        """
        Move all traffic vehicles
        :param dt: Decision keeping time
        :return: None
        """
        dt /= 3.6  # 1m/s = 3.6km/h
        for v in self.traffic_vehicles:
            v.step(dt)

    def update_state(self, scene_mgr, pg_world: PGWorld) -> bool:
        """
        Update all traffic vehicles' states,
        :param scene_mgr: Access other entities in the scene to determine the new state
        :param pg_world: World
        :return: if this episode should be done
        """

        vehicles_to_remove = []
        for v in self.traffic_vehicles:
            if v.out_of_road:
                remove = v.need_remove()
                if remove:
                    vehicles_to_remove.append(v)
                else:
                    # TODO traffic system will be updated soon
                    self.vehicles.remove(v.vehicle_node.kinematic_model)
                    v.reset()
                    self.vehicles.append(v.vehicle_node.kinematic_model)
            else:
                v.update_state(pg_world)

        # remove vehicles out of road
        for v in vehicles_to_remove:
            self.traffic_vehicles.remove(v)
            self.vehicles.remove(v.vehicle_node.kinematic_model)
            v.destroy(pg_world)

            if self.mode == TrafficMode.Hybrid:
                # create a new one
                lane = self.np_random.choice(self.reborn_lanes)
                vehicle_type = self.random_vehicle_type()
                random_v = self.spawn_one_vehicle(vehicle_type, lane, self.np_random.rand() * lane.length / 2, True)
                self.traffic_vehicles.append(random_v)

        return False

    def _clear_traffic(self, pg_world: PGWorld):
        if self._spawned_vehicles is not None:
            for v in self._spawned_vehicles:
                v.destroy(pg_world)

    def reset(self, pg_world: PGWorld, map: Map, controllable_vehicles: List, traffic_density: float) -> None:
        """
        Clear the scene and then reset the scene to empty
        :param pg_world: PGWorld class
        :param map: Map class containing road_network
        :param controllable_vehicles: a list of controllable vehicles
        :param traffic_density: the density of traffic in this episode
        :return: None
        """
        self._clear_traffic(pg_world)

        self.vehicles = []
        self.block_triggered_vehicles = [] if self.mode != TrafficMode.Reborn else None
        self.traffic_vehicles = deque()  # it is used to step all vehicles on scene
        self._spawned_vehicles = []

        logging.debug("load scene {}, {}".format(map.random_seed, "Use random traffic" if self.random_traffic else ""))
        self.update_random_seed(map.random_seed)
        if self.random_traffic:
            self.random_seed = None

        # single agent env
        self.ego_vehicle = controllable_vehicles[0] if len(controllable_vehicles) == 1 else None
        # TODO multi-agent env support
        self.controllable_vehicles = controllable_vehicles if len(controllable_vehicles) > 1 else None
        # update global info
        self.map = map
        self.density = traffic_density

        # update vehicle list
        self.vehicles.append(*controllable_vehicles)  # it is used to perform IDM and bicycle model based motion

    def get_vehicle_num(self):
        """
        Get the vehicles on road
        :return:
        """
        if self.mode == TrafficMode.Reborn:
            return len(self.traffic_vehicles)
        return sum(len(block_vehicle_set.vehicles) for block_vehicle_set in self.block_triggered_vehicles)

    def get_global_states(self) -> Dict:
        """
        Return all traffic vehicles' states
        :return: States of all vehicles
        """
        states = dict()
        for vehicle in self.traffic_vehicles:
            states[vehicle.index] = vehicle.get_state()

        # collect other vehicles
        if self.mode != TrafficMode.Reborn:
            for v_b in self.block_triggered_vehicles:
                for vehicle in v_b.vehicles:
                    states[vehicle.index] = vehicle.get_state()

        states["ego"] = self.ego_vehicle.get_state()
        return states

    def get_global_init_states(self) -> Dict:
        """
        Special handling for first states of traffic vehicles
        :return: States of all vehicles
        """
        vehicles = dict()
        for vehicle in self.traffic_vehicles:
            init_state = vehicle.get_state()
            init_state["index"] = vehicle.index
            init_state["type"] = vehicle.class_name
            init_state["enable_reborn"] = vehicle.enable_reborn
            vehicles[vehicle.index] = init_state

        # collect other vehicles
        if self.mode != TrafficMode.Reborn:
            for v_b in self.block_triggered_vehicles:
                for vehicle in v_b.vehicles:
                    init_state = vehicle.get_state()
                    init_state["type"] = vehicle.class_name
                    init_state["index"] = vehicle.index
                    init_state["enable_reborn"] = vehicle.enable_reborn
                    vehicles[vehicle.index] = init_state
        return vehicles

    def spawn_one_vehicle(self, vehicle_type, lane: AbstractLane, long: float, enable_reborn: bool):
        """
        Create one vehicle on lane and a specific place
        :param vehicle_type: PGTrafficVehicle type (s,m,l,xl)
        :param lane: Straight Lane or Circular Lane
        :param long: longitude position on lane
        :param enable_reborn: Reborn or not
        :return: PGTrafficVehicle
        """
        random_v = vehicle_type.create_random_traffic_vehicle(
            len(self.vehicles), self, lane, long, seed=self.random_seed, enable_reborn=enable_reborn
        )
        self._spawned_vehicles.append(random_v)
        self.vehicles.append(random_v.vehicle_node.kinematic_model)
        return random_v

    def _create_vehicles_on_lane(self, traffic_density: float, lane: AbstractLane, is_reborn_lane):
        """
        Create vehicles on a lane
        :param traffic_density: traffic density according to num of vehicles per meter
        :param lane: Circular lane or Straight lane
        :param is_reborn_lane: Whether vehicles should be reborn on this lane or not
        :return: List of vehicles
        """

        from pgdrive.scene_creator.blocks.ramp import InRampOnStraight
        traffic_vehicles = []
        total_num = int(lane.length / self.VEHICLE_GAP)
        vehicle_longs = [i * self.VEHICLE_GAP for i in range(total_num)]
        self.np_random.shuffle(vehicle_longs)
        for long in vehicle_longs:
            if self.np_random.rand() > traffic_density and abs(lane.length - InRampOnStraight.RAMP_LEN) > 0.1:
                # Do special handling for ramp, and there must be vehicles created there
                continue
            vehicle_type = self.random_vehicle_type()
            random_v = self.spawn_one_vehicle(vehicle_type, lane, long, is_reborn_lane)
            traffic_vehicles.append(random_v)
        return traffic_vehicles

    def _create_reborn_vehicles(self, pg_world: PGWorld, map: Map, traffic_density: float):
        reborn_lanes = self._get_available_reborn_lanes(map)
        for lane in reborn_lanes:
            self.traffic_vehicles += self._create_vehicles_on_lane(traffic_density, lane, True)
        for vehicle in self.traffic_vehicles:
            vehicle.attach_to_pg_world(pg_world.pbr_worldNP, pg_world.physics_world)

    def _create_vehicles_once(self, pg_world: PGWorld, map: Map, traffic_density: float) -> None:
        """
        Trigger mode, vehicles will be triggered only once, and disappear when arriving destination
        :param pg_world: World
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
            reborn_lanes = block.get_reborn_lanes()
            for lanes in reborn_lanes:
                if lanes not in trigger_lanes:
                    trigger_lanes.append(lanes)
            self.np_random.shuffle(trigger_lanes)
            for lanes in trigger_lanes:
                num = min(int(len(lanes) * traffic_density) + 1, len(lanes))
                lanes = self.np_random.choice(lanes, num, replace=False) if len(lanes) != 1 else lanes
                for l in lanes:
                    vehicles_on_block += self._create_vehicles_on_lane(traffic_density, l, False)
            for vehicle in vehicles_on_block:
                vehicle.attach_to_pg_world(pg_world.pbr_worldNP, pg_world.physics_world)
            block_vehicles = BlockVehicles(trigger_road=trigger_road, vehicles=vehicles_on_block)
            self.block_triggered_vehicles.append(block_vehicles)
            vehicle_num += len(vehicles_on_block)
        self.block_triggered_vehicles.reverse()

    @staticmethod
    def _get_available_reborn_lanes(map: Map) -> list:
        """
        Used to find some reborn lanes
        :param map: select born lanes from this map
        :return: reborn_lanes
        """
        reborn_lanes = []
        reborn_roads = []
        for block in map.blocks:
            roads = block.get_reborn_roads()
            for road in roads:
                if road in reborn_roads:
                    reborn_roads.remove(road)
                else:
                    reborn_roads.append(road)
        for road in reborn_roads:
            reborn_lanes += road.get_lanes(map.road_network)
        return reborn_lanes

    def close_vehicles_to(self, vehicle, distance: float, count: int = None, see_behind: bool = True) -> object:
        """
        Find the closest vehicles for IDM vehicles
        :param vehicle: IDM vehicle
        :param distance: How much distance
        :param count: Num of vehicles to return
        :param see_behind: Whether find vehicles behind this IDM vehicle or not
        :return:
        """
        vehicles = [
            v for v in self.vehicles
            if norm((v.position - vehicle.position)[0], (v.position - vehicle.position)[1]) < distance
            and v is not vehicle and (see_behind or -2 * vehicle.LENGTH < vehicle.lane_distance_to(v))
        ]

        vehicles = sorted(vehicles, key=lambda v: abs(vehicle.lane_distance_to(v)))
        if count:
            vehicles = vehicles[:count]
        return vehicles

    def neighbour_vehicles(self, vehicle, lane_index: Tuple = None) -> Tuple:
        """
        Find the preceding and following vehicles of a given vehicle.

        :param vehicle: the vehicle whose neighbours must be found
        :param lane_index: the lane on which to look for preceding and following vehicles.
                     It doesn't have to be the current vehicle lane but can also be another lane, in which case the
                     vehicle is projected on it considering its local coordinates in the lane.
        :return: its preceding vehicle, its following vehicle
        """
        lane_index = lane_index or vehicle.lane_index
        if not lane_index:
            return None, None
        lane = self.map.road_network.get_lane(lane_index)
        s = self.map.road_network.get_lane(lane_index).local_coordinates(vehicle.position)[0]
        s_front = s_rear = None
        v_front = v_rear = None
        for v in self.vehicles:
            if norm(v.position[0] - vehicle.position[0], v.position[1] - vehicle.position[1]) > 100:
                # coarse filter
                continue
            if v is not vehicle:
                s_v, lat_v = lane.local_coordinates(v.position)
                if not lane.on_lane(v.position, s_v, lat_v, margin=1):
                    continue
                if s <= s_v and (s_front is None or s_v <= s_front):
                    s_front = s_v
                    v_front = v
                if s_v < s and (s_rear is None or s_v > s_rear):
                    s_rear = s_v
                    v_rear = v
        return v_front, v_rear

    def random_vehicle_type(self):
        from pgdrive.scene_creator.pg_traffic_vehicle.traffic_vehicle_type import vehicle_type
        vehicle_type = vehicle_type[self.np_random.choice(list(vehicle_type.keys()), p=[0.2, 0.3, 0.3, 0.2])]
        return vehicle_type

    def destroy(self, pg_world: PGWorld) -> None:
        """
        Destory func, release resource
        :param pg_world: World
        :return: None
        """
        self._clear_traffic(pg_world)
        # current map
        self.map = None

        # traffic vehicle list
        self.ego_vehicle = None
        self.vehicles = None
        self.traffic_vehicles = None
        self.block_triggered_vehicles = None

        # traffic property
        self.mode = None
        self.random_traffic = None
        self.density = None

        # control randomness of traffic
        self.random_seed = None
        self.np_random = None

    def __del__(self):
        logging.debug("{} is destroyed".format(self.__class__.__name__))

    def __repr__(self):
        return self.vehicles.__repr__()
