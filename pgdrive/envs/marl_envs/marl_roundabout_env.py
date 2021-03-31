from pgdrive.envs.multi_agent_pgdrive import MultiAgentPGDrive

from pgdrive.scene_creator.blocks.roundabout import Roundabout
from pgdrive.utils import PGConfig


class MultiAgentRoundaboutEnv(MultiAgentPGDrive):
    @staticmethod
    def default_config() -> PGConfig:
        config = MultiAgentPGDrive.default_config()
        config["target_vehicle_configs"] = {}
        config.update(
            {
                "map": "O",
                "vehicle_config": {"born_longitude": 0, "born_lateral":0},
                # clear base config
                "target_vehicle_configs": {
                    "agent0": {
                        "born_lane_index": (Roundabout.node(1, 0, 0), Roundabout.node(1, 0, 1), 0),
                        # "show_lidar": True
                        # "show_side_detector": True
                    },
                    "agent1": {
                        # "show_lidar": True,
                        "born_lane_index": (Roundabout.node(1, 1, 0), Roundabout.node(1, 1, 1), 0),
                    },
                    "agent2": {
                        "born_lane_index": (Roundabout.node(1, 2, 0), Roundabout.node(1, 2, 1), 0),
                        # "show_lidar": True,
                    },
                    "agent3": {
                        # "show_lidar": True,
                        "born_lane_index": (Roundabout.node(1, 3, 0), Roundabout.node(1, 3, 1), 0),
                    }
                },
                "num_agents": 4,
                "map_config": {
                    "lane_num": 1},
            },
            allow_overwrite=True
        )
        return config

    def _after_lazy_init(self):
        super(MultiAgentRoundaboutEnv, self)._after_lazy_init()

        # target nodes contain a sequence of road nodes
        # they are only used to serve as "destination"
        # we will only make the point BEHIND the vehicles as the final destination, and we will generate a series of
        # checkpoints from current position to the destination automatically.
        self.target_nodes = [
            Roundabout.node(1, 0, 0),
            Roundabout.node(1, 0, 1),
            Roundabout.node(1, 1, 0),
            Roundabout.node(1, 1, 1),
            Roundabout.node(1, 2, 0),
            Roundabout.node(1, 2, 1),
            Roundabout.node(1, 3, 0),
            Roundabout.node(1, 3, 1),
        ]

    def step(self, actions):
        o, r, d, i = super(MultiAgentRoundaboutEnv, self).step(actions)
        self._update_target()
        return o, r, d, i

    def _update_target(self):
        for v_id, v in self.vehicles.items():
            if v.lane_index[0] in self.target_nodes:
                last_idx = self.target_nodes.index(v.lane_index[0]) - 2
                v.routing_localization.set_route(v.lane_index[0], self.target_nodes[last_idx])


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
