import time

from pgdrive import PGDriveEnv

if __name__ == '__main__':
    env = PGDriveEnv(dict(environment_num=1000, traffic_density=0.1, load_map_from_json=True, start_seed=5000))
    obs = env.reset()
    start = time.time()
    vc = []
    for s in range(1000):
        env.reset(force_seed=s + 5000)
        print("We have {} vehicles in seed {} map!".format(len(env.engine.traffic_manager.vehicles), s))
        vc.append(len(env.engine.traffic_manager.vehicles))
        if (s + 1) % 1 == 0:
            print(f"{s + 1} | Time Elapse: {time.time() - start}")
    import numpy as np
    print(np.mean(vc), np.min(vc), np.max(vc), np.std(vc))
