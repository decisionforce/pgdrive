from pgdrive.component.map.argoverse_map import ArgoverseMap
from pgdrive.envs.pgdrive_env import PGDriveEnv

RENDER = True
EXPERT = True

class ArgoverseEnv(PGDriveEnv):
    def _post_process_config(self, config):
        config = super(ArgoverseEnv, self)._post_process_config(config)
        config["traffic_mode"] = "Real"
        # config["vehicle_config"]["spawn_lane_index"] = ("11713", "4250", 0)
        config["vehicle_config"]["spawn_lane_index"] = ('7903', '9713', 0)
        config["vehicle_config"]["destination_node"] = "968"

        real_data_config = {
            "data_path":"/home/xzh/Research/code/argoverse-api/argoverse-tracking/sample",
            "log_id":"c6911883-1843-3727-8eaa-41dc8cda8993",
            "replay_agent": True,
        }
        config.update({"real_data_config": real_data_config})
        return config

    def _update_map(self, episode_data: dict = None):
        xcenter, ycenter = 2647.745783746136, 1253.652566359981
        if self.current_map is None:
            self.config["map_config"].update(
                {
                    "city": "PIT",
                    # "draw_map_resolution": 1024,
                    "center": ArgoverseMap.pgdrive_position([xcenter, ycenter]),
                    "radius": 100
                }
            )
            map = ArgoverseMap(self.config["map_config"])
            self.engine.map_manager.load_map(map)


class TestEnv(ArgoverseEnv):
    def __init__(self):
        super(TestEnv, self).__init__(
            {
                "traffic_density": 0.1,
                # "onscreen_message": True,
                # "debug_physics_world": True,
                "pstats": True,
                "global_light": True,
                # "debug_static_world":True,
                "cull_scene": False,
                # "controller":"joystick",
                "manual_control": RENDER and not EXPERT,
                "use_render": RENDER,
                "decision_repeat": 5,
                "rgb_clip": True,
                # "debug": False,
                "fast": False,
                "vehicle_config": {
                    "enable_reverse": True,
                #     "side_detector": dict(num_lasers=2, distance=50),
                #     "lane_line_detector": dict(num_lasers=2, distance=50),
                }
            }
        )


if __name__ == "__main__":
    env = TestEnv()
    from pgdrive.examples import expert

    o = env.reset()
    for i in range(1, 100000):
        action = expert(o) if EXPERT else [0, 0]
        o, r, d, info = env.step(action)
        info = {}
        info["position"] = env.vehicle.last_position
        # info["left_lane_index"] =
        # info["right_lane_index"]
        # if RENDER:
        #     env.render(text=info)
    env.close()
