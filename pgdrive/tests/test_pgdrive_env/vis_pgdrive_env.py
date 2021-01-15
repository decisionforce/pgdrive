from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.scene_creator.map import Map, MapGenerateMethod
from pgdrive.utils import setup_logger

setup_logger(True)


class TestEnv(PGDriveEnv):
    def __init__(self):
        """
        TODO a small bug exists in scene 9 (30 blocks)
        """
        super(TestEnv, self).__init__(
            {
                "environment_num": 1,
                "traffic_density": 0.3,
                "traffic_mode": "reborn",
                "start_seed": 5,
                "pg_world_config": {
                    "onscreen_message": True,
                    # "debug_physics_world":True,
                    "pstats": True
                },
                # "controller":"joystick",
                "image_source": "mini_map",
                "manual_control": True,
                "use_render": True,
                "use_image": False,
                "steering_penalty": 0.0,
                "decision_repeat": 5,
                "rgb_clip": True,
                # "debug":True,
                "map_config": {
                    Map.GENERATE_METHOD: MapGenerateMethod.BIG_BLOCK_NUM,
                    Map.GENERATE_PARA: 30,
                    Map.LANE_WIDTH: 3.5,
                    Map.LANE_NUM: 3,
                }
            }
        )


if __name__ == "__main__":
    env = TestEnv()

    o = env.reset()
    for i in range(1, 100000):
        o, r, d, info = env.step([0, 1])
        env.render()
        if d:
            print("Reset")
            env.reset()
    env.close()
