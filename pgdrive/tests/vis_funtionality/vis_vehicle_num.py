from pgdrive.envs.pgdrive_env_v2 import PGDriveEnvV2

if __name__ == "__main__":
    env = PGDriveEnvV2({
        "environment_num": 100,
        "start_seed": 5000,
        "traffic_density": 0.08,
    })
    env.reset()
    count = []
    for i in range(1, 101):
        o, r, d, info = env.step([0, 1])
        env.reset()
        print(
            "Current map {}, vehicle number {}.".format(
                env.current_seed, env.pgdrive_engine.traffic_manager.get_vehicle_num()
            )
        )
        count.append(env.pgdrive_engine.traffic_manager.get_vehicle_num())
    print(min(count), sum(count) / len(count), max(count))
    env.close()
