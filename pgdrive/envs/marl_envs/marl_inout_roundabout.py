from pgdrive.envs.multi_agent_pgdrive import MultiAgentPGDrive
from pgdrive.scene_creator.road.road import Road
from pgdrive.scene_creator.blocks.first_block import FirstBlock
from pgdrive.scene_creator.blocks.roundabout import Roundabout
from pgdrive.scene_creator.map import Map, MapGenerateMethod
from pgdrive.utils import get_np_random, PGConfig


class MultiAgentRoundaboutEnv(MultiAgentPGDrive):
    born_roads = [Road(FirstBlock.NODE_2, FirstBlock.NODE_3),
                  -Road(Roundabout.node(1, 0, 2), Roundabout.node(1, 0, 3)),
                  -Road(Roundabout.node(1, 1, 2), Roundabout.node(1, 1, 3)),
                  -Road(Roundabout.node(1, 2, 2), Roundabout.node(1, 2, 3)),
                  ]

    @staticmethod
    def default_config() -> PGConfig:
        config = MultiAgentPGDrive.default_config()
        config.update(
            {
                "map_config": {
                    Map.GENERATE_TYPE: MapGenerateMethod.BIG_BLOCK_SEQUENCE,
                    Map.GENERATE_CONFIG: "O",
                    Map.LANE_WIDTH: 3.5,
                    Map.LANE_NUM: 2,
                },
                "map": "O",
                "vehicle_config": {
                    "born_longitude": 5,
                    "born_lateral": 0,
                },
                # clear base config
                "num_agents": 1,
            },
            allow_overwrite=True
        )
        return config

    def _after_lazy_init(self):
        super(MultiAgentRoundaboutEnv, self)._after_lazy_init()
        if hasattr(self, "main_camera") and self.main_camera is not None:
            self.main_camera.camera.setPos(0, 0, 100)
            self.main_camera.stop_chase(self.pg_world)
            self.main_camera.camera_x += 60

    def _process_extra_config(self, config):
        config = super(MultiAgentRoundaboutEnv, self)._process_extra_config(config)
        config = self._update_agent_pos_configs(config)
        return super(MultiAgentRoundaboutEnv, self)._process_extra_config(config)

    def _update_agent_pos_configs(self, config):
        target_vehicle_configs = []
        assert config["num_agents"] <= config["map_config"]["lane_num"] * len(self.born_roads), (
            "Too many agents! We only accepet {} agents, but you have {} agents!".format(
                config["map_config"]["lane_num"] * len(self.target_nodes), config["num_agents"]
            )
        )
        for i, road in enumerate(self.born_roads):
            for lane_idx in range(config["map_config"]["lane_num"]):
                target_vehicle_configs.append(
                    (
                        "agent_{}_{}".format(i + 1, lane_idx),
                        road.lane_index(lane_idx)
                    )
                )
        target_agents = get_np_random().choice(
            [i for i in range(len(self.born_roads) * (config["map_config"]["lane_num"]))],
            config["num_agents"],
            replace=False
        )
        ret = {}
        for real_idx, idx in enumerate(target_agents):
            agent_name, v_config = target_vehicle_configs[idx]
            # for rllib compatibility
            ret["agent{}".format(real_idx)] = dict(born_lane_index=v_config)
        config["target_vehicle_configs"] = ret
        return config

    def step(self, actions):
        o, r, d, i = super(MultiAgentRoundaboutEnv, self).step(actions)
        return o, r, d, i

    def reset(self, episode_data: dict = None):
        ret = super(MultiAgentRoundaboutEnv, self).reset(episode_data)
        self._update_destination()
        return ret

    def _update_destination(self):
        # FIXME
        # when agent re-joined to the game, call this to set the new route to destination
        for v in self.vehicles.values():
            end_road = -get_np_random().choice(self.born_roads)
            v.routing_localization.set_route(v.lane_index[0], end_road.end_node)


if __name__ == "__main__":
    env = MultiAgentRoundaboutEnv(
        {
            "use_render": True,
            "debug": False,
            "manual_control": True,
            "pg_world_config": {
                "pstats": True
            },
            "crash_done": False,
            "num_agents": 4,
            "map_config": {
                Map.GENERATE_TYPE: MapGenerateMethod.BIG_BLOCK_SEQUENCE,
                Map.GENERATE_CONFIG: "O",
                Map.LANE_WIDTH: 3.5,
                Map.LANE_NUM: 2,
            }
        }
    )
    o = env.reset()
    # env.main_camera.set_follow_lane(True)
    total_r = 0
    for i in range(1, 100000):
        o, r, d, info = env.step(env.action_space.sample())
        for r_ in r.values():
            total_r += r_
        # o, r, d, info = env.step([0,1])dddd
        d.update({"total_r": total_r})
        env.render(text=d)
        if len(env.vehicles) == 0:
            total_r = 0
            print("Reset")
            env.reset()
    env.close()
