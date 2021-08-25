from pgdrive.component.map.argoverse_map import ArgoverseMap
from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.constants import ARGOVERSE_AGENT_ID
from pgdrive.utils.argoverse_utils import parse_tracking_data
import numpy as np

RENDER = True
EXPERT = True

class ArgoverseEnv(PGDriveEnv):
    def __init__(self, *args, **kwargs):
        data_path = "/home/xzh/Research/code/argoverse-api/argoverse-tracking/sample"
        # data_path = "/home/xzh/Research/code/argoverse-api/argoverse-tracking/train_parsed"
        # log_id = "10b3a1d8-e56c-38be-aaf7-ef2f862a5c4e"
        # log_id = "a073e840-6319-3f0b-843e-f6dccdcc7b77"
        log_id = "c6911883-1843-3727-8eaa-41dc8cda8993"
        data_parsed = False
        self.replay_agent = True

        if data_parsed:
            import pickle
            import os
            with open(os.path.join(data_path, "{}.pkl".format(log_id)), 'rb') as f:
                self.locate_info, self.city = pickle.load(f)
        else:
            self.locate_info, self.city = parse_tracking_data(data_path, log_id)

        self.map_center = self.locate_info[ARGOVERSE_AGENT_ID]["init_pos"] * np.array([1, -1])
        
        self.agent_locate_info = None
        if self.replay_agent:
            self.agent_locate_info = self.locate_info[ARGOVERSE_AGENT_ID]
            del self.locate_info[ARGOVERSE_AGENT_ID]
        super(ArgoverseEnv, self).__init__(*args, **kwargs)

    def _post_process_config(self, config):
        config = super(ArgoverseEnv, self)._post_process_config(config)
        config["traffic_mode"] = "Real"
        # config["vehicle_config"]["spawn_lane_index"] = ("11713", "4250", 0)
        # config["vehicle_config"]["spawn_lane_index"] = ('7903', '9713', 0)
        # config["vehicle_config"]["destination_node"] = "968"

        real_data_config = {
            "locate_info": self.locate_info,
            "agent_locate_info": self.agent_locate_info,
            "replay_agent": self.replay_agent
        }
        config.update({"real_data_config": real_data_config})
        return config

    def _update_map(self, episode_data: dict = None):
        # xcenter, ycenter = 2647.745783746136, 1253.652566359981
        if self.current_map is None:
            self.config["map_config"].update(
                {
                    "city": self.city,
                    # "draw_map_resolution": 1024,
                    "center": ArgoverseMap.pgdrive_position(self.map_center),
                    "radius": 150
                }
            )
            map = ArgoverseMap(self.config["map_config"])
            self.engine.map_manager.load_map(map)
            self.config["vehicle_config"]["spawn_lane_index"] = map.blocks[0].argo_lanes[0].index
            self.config["vehicle_config"]["destination_node"] = map.blocks[0].argo_lanes[-1].index[0]


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
