from pgdrive import PGDriveEnv
from pgdrive.scene_creator.highway_vehicle.behavior import IDMVehicle


class SidePassEnv(PGDriveEnv):
    def __init__(self, extra_config=None):
        config = {
            "environment_num": 1,
            "traffic_density": 0.1,
            "start_seed": 5,
            "pg_world_config": {
                "debug_physics_world": False,
            },
            "debug": False,
            "map": "CSC"
        }
        if extra_config is not None:
            config.update(extra_config)
        super(SidePassEnv, self).__init__(config)
        self.breakdown_vehicle = None
        self.alert = None

    def reset(self, episode_data: dict = None):
        ret = super(SidePassEnv, self).reset(episode_data)
        lane = self.current_map.road_network.graph[">>>"]["1C0_0_"][0]
        self.breakdown_vehicle = self.scene_manager.traffic_mgr.spawn_one_vehicle(
            self.scene_manager.traffic_mgr.random_vehicle_type(), lane, 30, False
        )
        self.breakdown_vehicle.attach_to_pg_world(self.pg_world.pbr_worldNP, self.pg_world.physics_world)
        self.alert = self.scene_manager.objects_mgr.spawn_one_object("Traffic Triangle", lane, 10, 0)
        self.alert.attach_to_pg_world(self.pg_world.pbr_worldNP, self.pg_world.physics_world)

        lane = self.current_map.road_network.graph["1C0_1_"]["2S0_0_"][2]
        pos = [(-20, lane.width / 3), (-15.6, lane.width / 4), (-12.1, 0),
               (-8.7, -lane.width / 4), (-4.2, -lane.width / 2), (-0.7, -lane.width * 3 / 4),
               (4.1, -lane.width), (7.3, -lane.width), (11.5, -lane.width),
               (15.5, -lane.width), (20.0, -lane.width), (23.2, -lane.width),
               (29.1, -lane.width), (32.9, -lane.width / 2), (37.0, 0),
               (40.0, lane.width / 2)]

        for p in pos:
            cone = self.scene_manager.objects_mgr.spawn_one_object("Traffic Cone", lane, *p)
            cone.attach_to_pg_world(self.pg_world.pbr_worldNP, self.pg_world.physics_world)

        v_pos = [8, 14]
        for v_long in v_pos:
            v = self.scene_manager.traffic_mgr.spawn_one_vehicle(
                self.scene_manager.traffic_mgr.random_vehicle_type(), lane, v_long, False
            )
            v.attach_to_pg_world(self.pg_world.pbr_worldNP, self.pg_world.physics_world)
        air_wall = IDMVehicle.create_random(self.scene_manager.traffic_mgr, lane, -25, 0, None)
        self.scene_manager.traffic_mgr.vehicles.append(air_wall)

        air_wall = IDMVehicle.create_random(self.scene_manager.traffic_mgr,
                                            self.current_map.road_network.graph["1C0_1_"]["2S0_0_"][1], -20, 0,
                                            None)
        self.scene_manager.traffic_mgr.vehicles.append(air_wall)

        return ret
