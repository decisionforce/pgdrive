from pgdrive.envs.multi_agent_pgdrive import MultiAgentPGDrive

from pgdrive.scene_creator.blocks.first_block import FirstBlock
from pgdrive.utils import PGConfig


class MultiAgentRoundaboutEnv(MultiAgentPGDrive):
    @staticmethod
    def default_config() -> PGConfig:
        config = MultiAgentPGDrive.default_config()
        config.update(
            {
                "map": "O",
                "target_vehicle_configs": {
                    "agent0": {
                        "born_longitude": 10,
                        "born_lateral": 1.5,
                        "born_lane_index": (FirstBlock.NODE_1, FirstBlock.NODE_2, 1),
                        # "show_lidar": True
                        # "show_side_detector": True
                    },
                    "agent1": {
                        "born_longitude": 10,
                        # "show_lidar": True,
                        "born_lateral": -1,
                        "born_lane_index": (FirstBlock.NODE_1, FirstBlock.NODE_2, 0),
                    },
                    "agent2": {
                        "born_longitude": 10,
                        "born_lane_index": (FirstBlock.NODE_1, FirstBlock.NODE_2, 2),
                        # "show_lidar": True,
                        "born_lateral": 1,
                    },
                    "agent3": {
                        "born_longitude": 10,
                        # "show_lidar": True,
                        "born_lateral": 2,
                        "born_lane_index": (FirstBlock.NODE_1, FirstBlock.NODE_2, 0),
                    }
                },
                "num_agents": 4,
            },
            allow_overwrite=True
        )
        # Some collision bugs still exist, always set to False now!!!!
        # config.extend_config_with_unknown_keys({"crash_done": True})
        return config


if __name__ == "__main__":
    env = MultiAgentRoundaboutEnv(
        {
            "use_render": True,
            "debug": False,
            "manual_control": True,
            "pg_world_config": {
                "pstats": False
            }
        }
    )
    o = env.reset()
    total_r = 0
    for i in range(1, 100000):
        o, r, d, info = env.step({"agent0": [-1, 0], "agent1": [0, 0], "agent2": [-1, 0], "agent3": [0, 0]})
        for r_ in r.values():
            total_r += r_
        # o, r, d, info = env.step([0,1])
        d.update({"total_r": total_r})
        env.render(text=d)
        if len(env.vehicles) == 0:
            total_r = 0
            print("Reset")
            env.reset()
    env.close()
