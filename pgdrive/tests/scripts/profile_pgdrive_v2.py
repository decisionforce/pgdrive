import time

from pgdrive import PGDriveEnvV2

if __name__ == '__main__':
    env = PGDriveEnvV2(dict(environment_num=1))
    obs = env.reset()
    start = time.time()
    action = [0.0, 0.1]
    for s in range(10000):
        o, r, d, i = env.step(action)
        if d:
            env.reset()
        if (s + 1) % 100 == 0:
            print(
                "Finish {}/10000 simulation steps. Time elapse: {:.4f}. Average FPS: {:.4f}".format(
                    s + 1,
                    time.time() - start, (s + 1) / (time.time() - start)
                )
            )
    print(f"(PGDriveEnvV2) Total Time Elapse: {time.time() - start}")
