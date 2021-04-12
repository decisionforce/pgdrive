from pgdrive.envs.multi_agent_pgdrive import MultiAgentPGDrive
from pgdrive.scene_creator.blocks.first_block import FirstBlock
from pgdrive.scene_creator.blocks.roundabout import Roundabout
from pgdrive.scene_creator.map import PGMap
from pgdrive.scene_creator.road.road import Road
from pgdrive.utils import get_np_random, PGConfig


class MARoundaboutMap(PGMap):
    def _generate(self, pg_world):
        length = 50

        parent_node_path, pg_physics_world = pg_world.worldNP, pg_world.physics_world
        assert len(self.road_network.graph) == 0, "These Map is not empty, please create a new map to read config"

        # Build a first-block
        last_block = FirstBlock(
            self.road_network, self.config[self.LANE_WIDTH], self.config[self.LANE_NUM], parent_node_path,
            pg_physics_world, 1, length=length
        )
        self.blocks.append(last_block)

        # Build roundabout
        Roundabout.EXIT_PART_LENGTH = length
        last_block = Roundabout(1, last_block.get_socket(index=0), self.road_network, random_seed=1)
        last_block.construct_block(parent_node_path, pg_physics_world, extra_config={
            "exit_radius": 10,
            "inner_radius": 30,
            "angle": 75
        })
        self.blocks.append(last_block)


class MultiAgentRoundaboutEnv(MultiAgentPGDrive):
    born_roads = [
        Road(FirstBlock.NODE_2, FirstBlock.NODE_3),
        -Road(Roundabout.node(1, 0, 2), Roundabout.node(1, 0, 3)),
        -Road(Roundabout.node(1, 1, 2), Roundabout.node(1, 1, 3)),
        -Road(Roundabout.node(1, 2, 2), Roundabout.node(1, 2, 3)),
        # -Road(Roundabout.node(1, 3, 2), Roundabout.node(1, 3, 3)),
        # -Road(Roundabout.node(1, 2, 2), Roundabout.node(1, 2, 3)),
    ]

    @staticmethod
    def default_config() -> PGConfig:
        config = MultiAgentPGDrive.default_config()
        config.update(
            {
                "camera_height": 4,
                "map": "M",
                "vehicle_config": {
                    "born_longitude": 5,
                    "born_lateral": 0,
                },
                # clear base config
                "num_agents": 1,
                "auto_termination": False
            },
            allow_overwrite=True,
        )
        return config

    def _update_map(self, episode_data: dict = None):
        if episode_data is not None:
            raise ValueError()
        map_config = self.config["map_config"]
        map_config.update({"seed": self.current_seed})

        if self.current_map is None:
            self.current_seed = 0
            new_map = MARoundaboutMap(self.pg_world, map_config)
            self.maps[self.current_seed] = new_map
            self.current_map = self.maps[self.current_seed]

    def _after_lazy_init(self):
        super(MultiAgentRoundaboutEnv, self)._after_lazy_init()

        # Use top-down view by default
        if hasattr(self, "main_camera") and self.main_camera is not None:
            self.main_camera.camera.setPos(0, 0, 120)
            self.main_camera.stop_chase(self.pg_world)
            self.main_camera.camera_x += 80
            self.main_camera.camera_y += 10

    def _process_extra_config(self, config):
        config = super(MultiAgentRoundaboutEnv, self)._process_extra_config(config)
        config = self._update_agent_pos_configs(config)
        return super(MultiAgentRoundaboutEnv, self)._process_extra_config(config)

    def _update_agent_pos_configs(self, config):
        target_vehicle_configs = []
        self.all_lane_index = []
        self.next_agent_id = config["num_agents"]
        # assert config["num_agents"] <= config["map_config"]["lane_num"] * len(self.born_roads), (
        #     "Too many agents! We only accepet {} agents, but you have {} agents!".format(
        #         config["map_config"]["lane_num"] * len(self.born_roads), config["num_agents"]
        #     )
        # )
        for i, road in enumerate(self.born_roads):
            for lane_idx in range(config["map_config"]["lane_num"]):
                target_vehicle_configs.append(("agent_{}_{}".format(i + 1, lane_idx), road.lane_index(lane_idx)))
                self.all_lane_index.append(road.lane_index(lane_idx))
        target_agents = get_np_random().choice(
            [i for i in range(len(self.born_roads) * (config["map_config"]["lane_num"]))],
            config["num_agents"],
            replace=True
        )
        ret = {}
        # for rllib compatibility
        if len(target_agents) > 1:
            for real_idx, idx in enumerate(target_agents):
                agent_name, v_config = target_vehicle_configs[idx]
                ret["agent{}".format(real_idx)] = dict(born_lane_index=v_config)
        else:
            agent_name, v_config = target_vehicle_configs[0]
            ret[self.DEFAULT_AGENT] = dict(born_lane_index=v_config)
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
            # use this line to always choose adverse exits!
            # v.routing_localization.set_route(v.lane_index[0], (-Road(*v.lane_index[:-1])).end_node)
            v.routing_localization.set_route(v.lane_index[0], end_road.end_node)

    def _after_vehicle_done(self, dones: dict):
        dones = self._wrap_as_multi_agent(dones)
        for id, done in dones.items():
            if done and id in self.vehicles.keys():
                if self.config["use_render"] and self.current_track_vehicle_id == id:
                    self.chase_another_v()
                new_id = "agent{}".format(self.next_agent_id)
                self.next_agent_id += 1
                v = self.vehicles.pop(id)
                obs = self.observations.pop(id)
                self.observations[new_id] = obs
                self.action_space = self._get_action_space()
                self.observation_space = self._get_observation_space()
                born_lane_index = get_np_random().choice(len((self.all_lane_index)), 1)[0]
                v.vehicle_config["born_lane_index"] = self.all_lane_index[born_lane_index]
                v.reset(self.current_map)
                self.vehicles[new_id] = v
                self.dones[new_id] = False


if __name__ == "__main__":
    env = MultiAgentRoundaboutEnv(
        {
            "fast": True,
            # "use_render": True,
            "debug": True,
            "manual_control": True,
            "num_agents": 1,
        }
    )
    o = env.reset()
    # env.main_camera.set_follow_lane(True)
    total_r = 0
    for i in range(1, 100000):
        o, r, d, info = env.step(env.action_space.sample())
        if env.num_agents == 1:
            r = env._wrap_as_multi_agent(r)
        for r_ in r.values():
            total_r += r_
        # o, r, d, info = env.step([0,1])dddd
        # d.update({"total_r": total_r})
        # env.render(text=d)
        if len(env.vehicles) == 0:
            total_r = 0
            print("Reset")
            env.reset()
    env.close()
