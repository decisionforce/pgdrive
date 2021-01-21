import logging
from collections import namedtuple, deque
from typing import Tuple, Dict

from pgdrive.scene_creator.lanes.lane import AbstractLane
from pgdrive.scene_creator.map import Map
from pgdrive.scene_creator.road.road import Road
from pgdrive.utils import norm, get_np_random
from pgdrive.world.pg_world import PGWorld
from pgdrive.scene_manager.PGLOD import PGLOD

BlockVehicles = namedtuple("block_vehicles", "trigger_road vehicles")


class TrafficMode:
    # Traffic vehicles will be reborn, once they arrive at the destinations
    Reborn = "reborn"
    # Traffic vehicles will be triggered only once
    Trigger = "trigger"


class TrafficManager:
    VEHICLE_GAP = 10  # m

    def __init__(self, traffic_mode: TrafficMode, random_traffic: bool):
        """
        Control the whole traffic flow
        :param traffic_mode: Reborn mode or Trigger mode
        :param random_traffic: the distribution of vehicles will be different in different episdoes
        """
        # current map
        self.map = None

        # traffic vehicle list
        self.ego_vehicle = None
        self.vehicles = None
        self.traffic_vehicles = None
        self.block_triggered_vehicles = None

        # traffic property
        self.mode = traffic_mode
        self.random_traffic = random_traffic
        self.density = 0

        # control randomness of traffic
        self.random_seed = None
        self.np_random = None

    def generate(self, pg_world: PGWorld, map: Map, ego_vehicle, traffic_density: float):
        """
        Generate traffic on map
        :param pg_world: World
        :param map: The map class containing block list and road network
        :param ego_vehicle: Ego vehicle
        :param traffic_density: Traffic density defined by: number of vehicles per meter
        :return: List of Traffic vehicles
        """
        logging.debug("load scene {}, {}".format(map.random_seed, "Use random traffic" if self.random_traffic else ""))
        self.update_random_seed(map.random_seed)

        # clear traffic in last episdoe
        self.clear_traffic(pg_world)

        # update global info
        self.ego_vehicle = ego_vehicle
        self.map = map
        self.density = traffic_density

        # update vehicle list
        self.block_triggered_vehicles = [] if self.mode == TrafficMode.Trigger else None
        self.vehicles = [ego_vehicle]  # it is used to perform IDM and bicycle model based motion
        self.traffic_vehicles = deque()  # it is used to step all vehicles on scene

        if abs(traffic_density - 0.0) < 1e-2:
            return

        if self.mode == TrafficMode.Reborn:
            # add reborn vehicle
            reborn_lanes = self._get_available_reborn_lanes(map)
            for lane in reborn_lanes:
                self.traffic_vehicles += self._create_vehicles_on_lane(traffic_density, lane, True)
            for vehicle in self.traffic_vehicles:
                vehicle.attach_to_pg_world(pg_world.pbr_worldNP, pg_world.physics_world)
            logging.debug("Init {} Traffic Vehicles".format(len(self.traffic_vehicles)))
        else:
            self._create_vehicles_once(pg_world, map, traffic_density)

    def prepare_step(self, scene_mgr):
        """
        All traffic vehicles make driving decision here
        :param scene_mgr: access other elements in scene
        :return: None
        """
        if self.mode == TrafficMode.Trigger:
            ego_lane_idx = self.ego_vehicle.lane_index[:-1]
            ego_road = Road(ego_lane_idx[0], ego_lane_idx[1])
            if len(self.block_triggered_vehicles) > 0 and ego_road == self.block_triggered_vehicles[-1].trigger_road:
                block_vehicles = self.block_triggered_vehicles.pop()
                self.traffic_vehicles += block_vehicles.vehicles
        for v in self.traffic_vehicles:
            v.prepare_step()

    def step(self, dt: float):
        """
        Move all traffic vehicles
        :param dt: Decision keeping time
        :return: None
        """
        dt /= 3.6  # 1m/s = 3.6km/h
        for v in self.traffic_vehicles:
            v.step(dt)

    def update_state(self, pg_world: PGWorld) -> bool:
        """
        Update all traffic vehicles' states,
        :param pg_world: World
        :return: if this episode should be done
        """
        # cull distant objects
        PGLOD.cull_distant_blocks(self.map.blocks, self.ego_vehicle.position, pg_world)
        PGLOD.cull_distant_traffic_vehicles(self.traffic_vehicles, self.ego_vehicle.position, pg_world)

        vehicles_to_remove = []
        for v in self.traffic_vehicles:
            if v.out_of_road:
                remove = v.need_remove()
                if remove:
                    vehicles_to_remove.append(v)
            else:
                v.update_state()

        # remove vehicles out of road
        for v in vehicles_to_remove:
            self.traffic_vehicles.remove(v)
            self.vehicles.remove(v.vehicle_node.kinematic_model)
            v.destroy(pg_world)
        return False

    def clear_traffic(self, pg_world: PGWorld) -> None:
        """
        Clear all traffic vehicles in map
        :param pg_world: World
        :return: None
        """
        if self.traffic_vehicles is not None:
            for v in self.traffic_vehicles:
                v.destroy(pg_world)
        if self.block_triggered_vehicles is not None:
            for block_vs in self.block_triggered_vehicles:
                for v in block_vs.vehicles:
                    v.destroy(pg_world)
        self.traffic_vehicles = None
        self.vehicles = None
        self.block_triggered_vehicles = None

    def update_random_seed(self, random_seed: int):
        """
        Update the random seed and random engine of traffic
        :param random_seed: int, random seed
        :return: None
        """
        self.random_seed = random_seed if not self.random_traffic else None
        self.np_random = get_np_random(self.random_seed)

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
        if self.mode == TrafficMode.Trigger:
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
            vehicles[vehicle.index] = init_state

        # collect other vehicles
        if self.mode == TrafficMode.Trigger:
            for v_b in self.block_triggered_vehicles:
                for vehicle in v_b.vehicles:
                    init_state = vehicle.get_state()
                    init_state["type"] = vehicle.class_name
                    init_state["index"] = vehicle.index
                    vehicles[vehicle.index] = init_state
        return vehicles

    def _create_vehicles_on_lane(self, traffic_density: float, lane: AbstractLane, is_reborn_lane):
        """
        Create vehicles on a lane
        :param traffic_density: traffic density according to num of vehicles per meter
        :param lane: Circular lane or Straight lane
        :param is_reborn_lane: Whether vehicles should be reborn on this lane or not
        :return: List of vehicles
        """
        from pgdrive.scene_creator.pg_traffic_vehicle.traffic_vehicle_type import car_type
        from pgdrive.scene_creator.blocks.ramp import InRampOnStraight
        traffic_vehicles = []
        total_num = int(lane.length / self.VEHICLE_GAP)
        vehicle_longs = [i * self.VEHICLE_GAP for i in range(total_num)]
        self.np_random.shuffle(vehicle_longs)
        for i, long in enumerate(vehicle_longs):
            if self.np_random.rand() > traffic_density and abs(lane.length - InRampOnStraight.RAMP_LEN) > 0.1:
                # Do special handling for ramp, and there must be vehicles created there
                continue
            vehicle_type = car_type[self.np_random.choice(list(car_type.keys()), p=[0.2, 0.3, 0.3, 0.2])]
            random_v = vehicle_type.create_random_traffic_vehicle(
                len(self.vehicles), self, lane, long, seed=self.random_seed, enable_reborn=is_reborn_lane
            )
            self.vehicles.append(random_v.vehicle_node.kinematic_model)
            traffic_vehicles.append(random_v)
        return traffic_vehicles

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
            vehicles_on_block = []
            trigger_road = block._pre_block_socket.positive_road

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
        logging.debug("Init {} Traffic Vehicles".format(vehicle_num))
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

    def destroy(self, pg_world: PGWorld) -> None:
        """
        Destory func, release resource
        :param pg_world: World
        :return: None
        """
        self.clear_traffic(pg_world)
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
