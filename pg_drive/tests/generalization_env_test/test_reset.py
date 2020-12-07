from pg_drive.envs.generalization_racing import GeneralizationRacing
from pg_drive.utils import setup_logger

setup_logger(debug=True)


class ResetEnv(GeneralizationRacing):
    def __init__(self):
        super(ResetEnv, self).__init__(
            {
                "environment_num": 1,
                "traffic_density": 0.1,
                "start_seed": 4,
                "debug": True,
                "pg_world_config": {
                    "force_fps": None,
                    "debug_physics_world": True
                },
                "manual_control": True,
                "use_render": True,
                "use_rgb": False,
                "use_increment_steering": False,
                # "map_config":{
                #     "type":BigGenerateMethod.BLOCK_SEQUENCE,
                #     "config":"",
                # }
            }
        )


if __name__ == "__main__":
    env = ResetEnv()
    import time

    env.reset()
    env.pg_world.accept("r", env.reset)
    t = 0.0
    for i in range(1, 200000):
        start = time.time()
        # print("Step: ", i)
        o, r, d, info = env.step([0.1, 0])
        t += time.time() - start
        # print(len(o), "Vs.", env.observation_space.shape[0])
        # print(info)
        env.render(text={"can you see me": i})
        # if i % 100 == 0:
        #     print(t / 50)
        #     t = 0.0
        #     env.reset()
        if d:
            env.reset()
    env.close()
