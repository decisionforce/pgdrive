import time

import numpy as np

from pg_drive import GeneralizationRacing

if __name__ == '__main__':
    # env = GeneralizationRacing(dict(environment_num=100))
    env = GeneralizationRacing(dict(environment_num=100, load_map_from_json=False))
    obs = env.reset()
    start = time.time()
    total = 1000
    for s in range(total):
        env.reset()
        # print("Finish")
        if (s + 1) % 1 == 0:
            print(f"{s + 1}/{total} Time Elapse: {time.time() - start}")
    print(f"Total Time Elapse: {time.time() - start}")
