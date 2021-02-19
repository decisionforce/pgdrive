from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import setup_logger

# setup_logger(True)


class ObjectTestEnv(PGDriveEnv):
    def __init__(self, extra_config=None):
        config = {
            "environment_num": 1,
            "traffic_density": 0.0,
            "start_seed": 5,
            "pg_world_config": {
                "debug_physics_world": False,
            },
            "debug": True,
            "map": "CS"
        }
        if extra_config is not None:
            config.update(extra_config)
        super(ObjectTestEnv, self).__init__(config)
        self.breakdown_vehicle = None
        self.alert = None

    def reset(self, episode_data: dict = None):
        ret = super(ObjectTestEnv, self).reset(episode_data)
        self.breakdown_vehicle = env.scene_manager.traffic_mgr.spawn_one_vehicle(
            env.scene_manager.traffic_mgr.random_vehicle_type(), env.vehicle.lane, 30, False
        )
        self.breakdown_vehicle.attach_to_pg_world(env.pg_world.pbr_worldNP, env.pg_world.physics_world)
        self.alert = env.scene_manager.objects_mgr.spawn_one_object("Traffic Cone", env.vehicle.lane, 20, 0)
        self.alert.attach_to_pg_world(env.pg_world.worldNP, env.pg_world.physics_world)
        return ret


if __name__ == "__main__":
    env = ObjectTestEnv({"manual_control": True, "use_render": True})
    o = env.reset()
    for i in range(1, 100000):
        o, r, d, info = env.step([0, 1])
        env.render()
        if d:
            env.reset()
    env.close()
