from pgdrive import PGDriveEnv


class SidePassEnv(PGDriveEnv):
    def __init__(self, extra_config=None):
        config = {
            "environment_num": 1,
            "traffic_density": 0.0,
            "start_seed": 5,
            "pg_world_config": {
                "debug_physics_world": False,
            },
            "debug": False,
            "map": "CS"
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
        self.alert = self.scene_manager.objects_mgr.spawn_one_object("Traffic Triangle", lane, 20, 0)
        self.alert.attach_to_pg_world(self.pg_world.pbr_worldNP, self.pg_world.physics_world)
        return ret
