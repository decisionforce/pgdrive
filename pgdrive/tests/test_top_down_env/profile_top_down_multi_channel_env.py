import time

from pgdrive import TopDownPGDriveEnv

if __name__ == '__main__':
    env = TopDownPGDriveEnv(dict(environment_num=10, frame_stack=10, frame_skip=3))
    o = env.reset()
    start = time.time()
    action = [0.0, 0.1]
    print(o.shape)
    for s in range(10000):
        o, r, d, i = env.step(action)
        if d:
            env.reset()
        if (s + 1) % 100 == 0:
            print(
                "(TopDownEnv) Finish {}/10000 simulation steps. Time elapse: {:.4f}. Average FPS: {:.4f}".format(
                    s + 1,
                    time.time() - start, (s + 1) / (time.time() - start)
                )
            )
    print(f"(TopDownEnv) Total Time Elapse: {time.time() - start}")
