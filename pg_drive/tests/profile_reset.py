import time

import numpy as np

from pg_drive import GeneralizationRacing

if __name__ == '__main__':
    env = GeneralizationRacing(
        dict(environment_num=100, load_map_from_json=True, use_render=True, manual_control=True, pg_world_config={"debug_physics_world": True}))
    # env = GeneralizationRacing(dict(environment_num=100, load_map_from_json=False))
    obs = env.reset()
    env.pg_world.accept("r", env.reset)
    start = time.time()
    total = 10000000

    for s in range(total):
        o,r,d,*_ = env.step([0.0, 0.0])
        env.render()
        if d:
            env.reset()
        # print("Finish")
    #     if (s + 1) % 1 == 0:
    #         print(f"{s + 1}/{total} Time Elapse: {time.time() - start}")
    # print(f"Total Time Elapse: {time.time() - start}")
