from pgdrive.world.pg_world import PGWorld

class TrafficMode:

    Reborn = "reborn"
    Trigger = "trigger"


class Traffic:

    @staticmethod
    def generate(pg_world, traffic_density:float,):
        if abs(self.traffic_density - 0.0) < 1e-2:
            return
        if self.traffic_mode == TrafficMode.Reborn:
            # add reborn vehicle
            for lane in self.reborn_lanes:
                self.traffic_vehicles += self._create_vehicles_on_lane(lane, True)
            for vehicle in self.traffic_vehicles:
                vehicle.attach_to_pg_world(pg_world.pbr_worldNP, pg_world.physics_world)
            logging.debug("Init {} Traffic Vehicles".format(len(self.traffic_vehicles)))
        else:
            self._create_vehicles_once(pg_world)

    def _create_vehicles_on_lane(self, lane, is_reborn_lane=False):
        """
        Create vehicles on one lane
        :param lane: Straight lane or Circular lane
        :param is_reborn_lane: Vehicles will be reborn when set to True
        :return: None
        """
        from pgdrive.scene_creator.pg_traffic_vehicle.traffic_vehicle_type import car_type
        from pgdrive.scene_creator.blocks.ramp import InRampOnStraight
        traffic_vehicles = []
        total_num = int(lane.length / self.VEHICLE_GAP)
        vehicle_longs = [i * self.VEHICLE_GAP for i in range(total_num)]
        self.np_random.shuffle(vehicle_longs)
        for i, long in enumerate(vehicle_longs):
            if self.np_random.rand() > self.traffic_density and abs(lane.length - InRampOnStraight.RAMP_LEN) > 0.1:
                # Do special handling for ramp, and there must be vehicles created there
                continue
            vehicle_type = car_type[self.np_random.choice(list(car_type.keys()), p=[0.2, 0.3, 0.3, 0.2])]
            random_v = vehicle_type.create_random_traffic_vehicle(
                len(self.vehicles), self, lane, long, seed=self.random_seed, enable_reborn=is_reborn_lane
            )
            self.vehicles.append(random_v.vehicle_node.kinematic_model)
            traffic_vehicles.append(random_v)
        return traffic_vehicles

    def _create_vehicles_once(self, pg_world):
        vehicle_num = 0
        for block in self.blocks[1:]:
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
                num = min(int(len(lanes) * self.traffic_density) + 1, len(lanes))
                lanes = self.np_random.choice(lanes, num, replace=False) if len(lanes) != 1 else lanes
                for l in lanes:
                    vehicles_on_block += self._create_vehicles_on_lane(l)
            for vehicle in vehicles_on_block:
                vehicle.attach_to_pg_world(pg_world.pbr_worldNP, pg_world.physics_world)
            block_vehicles = BlockVehicles(trigger_road=trigger_road, vehicles=vehicles_on_block)
            self.block_triggered_vehicles.append(block_vehicles)
            vehicle_num += len(vehicles_on_block)
        logging.debug("Init {} Traffic Vehicles".format(vehicle_num))
        self.block_triggered_vehicles.reverse()
