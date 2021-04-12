from pgdrive.envs.pgdrive_env_v2 import PGDriveEnvV2
from pgdrive.scene_creator.map import Map, MapGenerateMethod

if __name__ == "__main__":
    env = PGDriveEnvV2(
        {
            "environment_num": 10000,
            # "traffic_density": 0.1,
            # "traffic_mode": 0,  # 0 for Reborn mode.
            # "map_config": {
            #     Map.GENERATE_TYPE: MapGenerateMethod.BIG_BLOCK_NUM,
            #     Map.GENERATE_CONFIG: 7,
            # }
        }
    )
    env.reset()
    for i in range(1, 100000):
        o, r, d, info = env.step([0, 1])
        env.reset()
        print(
            "Current map {}, vehicle number {}.".format(
                env.current_seed, env.scene_manager.traffic_mgr.get_vehicle_num()
            )
        )
    env.close()
