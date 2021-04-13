import logging

import numpy as np
from pgdrive.envs.multi_agent_pgdrive import MultiAgentPGDrive
from pgdrive.scene_creator.blocks.first_block import FirstBlock
from pgdrive.scene_creator.blocks.roundabout import Roundabout
from pgdrive.scene_creator.map import PGMap
from pgdrive.scene_creator.road.road import Road
from pgdrive.utils import get_np_random, PGConfig, distance_greater


class MARoundaboutMap(PGMap):
    def _generate(self, pg_world):
        length = MultiAgentRoundaboutEnv.EXIT_LENGTH

        parent_node_path, pg_physics_world = pg_world.worldNP, pg_world.physics_world
        assert len(self.road_network.graph) == 0, "These Map is not empty, please create a new map to read config"

        # Build a first-block
        last_block = FirstBlock(
            self.road_network,
            self.config[self.LANE_WIDTH],
            self.config[self.LANE_NUM],
            parent_node_path,
            pg_physics_world,
            1,
            length=length
        )
        self.blocks.append(last_block)

        # Build roundabout
        Roundabout.EXIT_PART_LENGTH = length
        last_block = Roundabout(1, last_block.get_socket(index=0), self.road_network, random_seed=1)
        last_block.construct_block(
            parent_node_path,
            pg_physics_world,
            extra_config={
                "exit_radius": 10,
                "inner_radius": 40,
                "angle": 70,
                # Note: lane_num is set in config.map_config.lane_num
            }
        )
        self.blocks.append(last_block)


class BornPlaceManager:
    def __init__(self, safe_born_places: list):
        self.safe_born_places = {i: v for i, v in enumerate(safe_born_places)}
        self.mapping = {i: set() for i in self.safe_born_places.keys()}
        self.need_update_born_places = True

    def update(self, vehicles: dict, map):
        if self.need_update_born_places:
            self.need_update_born_places = False
            for bid, bp in self.safe_born_places.items():
                lane = map.road_network.get_lane(bp["config"]["born_lane_index"])
                self.safe_born_places[bid]["position"] = lane.position(longitudinal=bp["config"]["born_longitude"],
                                                                       lateral=bp["config"]["born_lateral"])
                for vid in vehicles.keys():
                    self.new(bid, vid)  # Just assume everyone is all in the same born place at t=0.

        for bid, vid_set in self.mapping.items():
            removes = []
            for vid in vid_set:
                if (vid not in vehicles) or (
                        distance_greater(self.safe_born_places[bid]["position"], vehicles[vid].position, length=10)):
                    removes.append(vid)
            for vid in removes:
                self.mapping[bid].remove(vid)
            # if removes:
            #     print("Remove vehicle {} in BP {}".format(removes, bid))

    def new(self, born_place_id, vehicle_id):
        self.mapping[born_place_id].add(vehicle_id)

    def get_available_born_places(self):
        ret = {}
        for bid in self.safe_born_places.keys():
            if not self.mapping[bid]:  # empty
                ret[bid] = self.safe_born_places[bid]
        return ret


class MultiAgentRoundaboutEnv(MultiAgentPGDrive):
    EXIT_LENGTH = 100
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
                "horizon": 1000,
                "camera_height": 4,
                "vehicle_config": {
                    "lidar": {
                        "num_lasers": 120,
                        "distance": 50,
                        "num_others": 4,
                    },
                    "show_lidar": False,
                    "born_longitude": 5,
                    "born_lateral": 0,
                },
                "map_config": {
                    "lane_num": 3
                },
                # clear base config
                "num_agents": 2,
                "auto_termination": False,

                # reward scheme
                "out_of_road_penalty": 5.0,
                "crash_vehicle_penalty": 1.0,
                "crash_object_penalty": 1.0,
            },
            allow_overwrite=True,
        )
        return config

    def _update_map(self, episode_data: dict = None, force_seed=None):
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
            bird_camera_height = 240
            self.main_camera.camera.setPos(0, 0, bird_camera_height)
            self.main_camera.bird_camera_height = bird_camera_height
            self.main_camera.stop_chase(self.pg_world)
            # self.main_camera.camera.setPos(300, 20, bird_camera_height)
            self.main_camera.camera_x += 140
            self.main_camera.camera_y += 20

    def _process_extra_config(self, config):
        config = super(MultiAgentRoundaboutEnv, self)._process_extra_config(config)
        config = self._update_agent_pos_configs(config)
        return super(MultiAgentRoundaboutEnv, self)._process_extra_config(config)

    def _update_agent_pos_configs(self, config):
        target_vehicle_configs = []
        self._all_lane_index = []
        self._next_agent_id = config["num_agents"]

        num_concurrent = 3
        assert config["num_agents"] <= config["map_config"]["lane_num"] * len(self.born_roads) * num_concurrent, (
            "Too many agents! We only accepet {} agents, but you have {} agents!".format(
                config["map_config"]["lane_num"] * len(self.born_roads) * num_concurrent, config["num_agents"]
            )
        )

        # We can spawn agents in the middle of road at the initial time, but when some vehicles need to be reborn,
        # then we have to set it to the farthest places to ensure safety (otherwise the new vehicles may suddenly
        # appear at the middle of the road!)
        safe_born_places = []
        self._last_born_identifier = None
        for i, road in enumerate(self.born_roads):
            for lane_idx in range(config["map_config"]["lane_num"]):
                for j in range(num_concurrent):
                    interval = self.EXIT_LENGTH / num_concurrent
                    long = j * interval + np.random.uniform(0, 0.5 * interval)
                    target_vehicle_configs.append(
                        (
                            "agent_{}_{}".format(i + 1, lane_idx), {
                                "born_lane_index": road.lane_index(lane_idx),
                                "born_longitude": long,
                                "born_lateral": config["vehicle_config"]["born_lateral"]
                            }
                        )
                    )
                    self._all_lane_index.append(road.lane_index(lane_idx))
                    if j == 0:
                        lane_tuple = road.lane_index(lane_idx)  # like (>>>, 1C0_0_, 1) and so on.
                        safe_born_places.append(
                            dict(
                                identifier=lane_tuple[0],  # identifier
                                config={
                                    "born_lane_index": lane_tuple,
                                    "born_longitude": long,
                                    "born_lateral": config["vehicle_config"]["born_lateral"]
                                },
                            )
                        )
        self._born_places_manager = BornPlaceManager(safe_born_places)

        target_agents = get_np_random().choice(
            [i for i in range(len(target_vehicle_configs))], config["num_agents"], replace=False
        )

        # for rllib compatibility
        ret = {}
        if len(target_agents) > 1:
            for real_idx, idx in enumerate(target_agents):
                agent_name, v_config = target_vehicle_configs[idx]
                ret["agent{}".format(real_idx)] = v_config
        else:
            agent_name, v_config = target_vehicle_configs[0]
            ret["agent0"] = v_config
        config["target_vehicle_configs"] = ret
        return config

    def reset(self, episode_data: dict = None, force_seed=None):
        self._next_agent_id = self.num_agents
        self._last_born_identifier = 0
        self._do_not_reborn = False
        ret = super(MultiAgentRoundaboutEnv, self).reset(episode_data)
        assert len(self.vehicles) == self.num_agents
        self.for_each_vehicle(self._update_destination_for)
        return ret

    def step(self, actions):
        o, r, d, i = super(MultiAgentRoundaboutEnv, self).step(actions)
        if self.episode_steps >= self.config["horizon"]:
            self._do_not_reborn = True

        # Check
        condition = set(kkk for kkk, rrr in r.items() if rrr == -self.config["out_of_road_penalty"]) == \
                    set(kkk for kkk, ddd in d.items() if ddd) == \
                    set(kkk for kkk, iii in i.items() if iii.get("out_of_road"))
        if not condition:
            raise ValueError("Observation not aligned!")

        # Update reborn manager
        self._born_places_manager.update(self.vehicles, self.current_map)

        # Fulfill __all__
        d["__all__"] = (
                ((self.episode_steps >= self.config["horizon"]) and (all(d.values()))) or (len(self.vehicles) == 0)
                or (self.episode_steps >= 5 * self.config["horizon"])
        )
        if d["__all__"]:
            for k in d.keys():
                d[k] = True
        return o, r, d, i

    def _update_destination_for(self, vehicle):
        # when agent re-joined to the game, call this to set the new route to destination
        end_road = -get_np_random().choice(self.born_roads)  # Use negative road!
        vehicle.routing_localization.set_route(vehicle.lane_index[0], end_road.end_node)

    def _reborn(self, dead_vehicle_id):
        assert dead_vehicle_id in self.vehicles, (dead_vehicle_id, self.vehicles.keys())
        # Switch to track other vehicle if in first-person view.
        # if self.config["use_render"] and self.current_track_vehicle_id == id:
        #     self.chase_another_v()

        v = self.vehicles.pop(dead_vehicle_id)
        v.prepare_step([0, -1])

        # register vehicle
        new_id = "agent{}".format(self._next_agent_id)
        self._next_agent_id += 1
        self.vehicles[new_id] = v  # Put it to new vehicle id.
        self.dones[new_id] = False  # Put it in the internal dead-tracking dict.
        logging.debug("{} Dead. {} Reborn!".format(dead_vehicle_id, new_id))

        # replace vehicle to new born place
        safe_places_dict = self._born_places_manager.get_available_born_places()
        # safe_places = [p for p in self._safe_born_places if p['identifier'] != self._last_born_identifier]
        bp_index = get_np_random().choice(list(safe_places_dict.keys()), 1)[0]
        new_born_place = safe_places_dict[bp_index]
        self._born_places_manager.new(born_place_id=bp_index, vehicle_id=new_id)

        new_born_place_config = new_born_place["config"]
        self._last_born_identifier = new_born_place["identifier"]
        v.vehicle_config.update(new_born_place_config)
        v.reset(self.current_map)
        v.update_state()

        # reset observation space
        obs = self.observations[dead_vehicle_id]
        self.observations[new_id] = obs
        self.observations[new_id].reset(self, v)

        new_obs = self.observations[new_id].observe(v)
        self.observation_space.spaces[new_id] = self.observation_space.spaces[dead_vehicle_id]
        old_act_space = self.action_space.spaces.pop(dead_vehicle_id)
        self.action_space.spaces[new_id] = old_act_space
        return new_obs, new_id

    def _after_vehicle_done(self, obs=None, reward=None, dones: dict = None, info=None):
        new_dones = dict()
        for dead_vehicle_id, done in dones.items():
            new_dones[dead_vehicle_id] = done
            if done and self._do_not_reborn:
                v = self.vehicles.pop(dead_vehicle_id)
                v.prepare_step([0, -1])
                self.done_vehicles[dead_vehicle_id] = v
                self.action_space.spaces.pop(dead_vehicle_id)
            if done and (not self._do_not_reborn):
                new_obs, new_id = self._reborn(dead_vehicle_id)
                obs[new_id] = new_obs
                reward[new_id] = 0.0
                info[new_id] = {}
                new_dones[new_id] = False
        return obs, reward, new_dones, info


def _draw():
    env = MultiAgentRoundaboutEnv()
    o = env.reset()
    from pgdrive.utils import draw_top_down_map
    import matplotlib.pyplot as plt

    plt.imshow(draw_top_down_map(env.current_map))
    plt.show()
    env.close()


def _vis():
    env = MultiAgentRoundaboutEnv(
        {
            "vehicle_config": {
                "lidar": {
                    "num_lasers": 120,
                    "distance": 50
                }
            },
            "fast": True,
            "use_render": True,
            "debug": True,
            "manual_control": True,
            "num_agents": 8,
        }
    )
    o = env.reset()
    total_r = 0
    ep_s = 0
    for i in range(1, 100000):
        o, r, d, info = env.step(env.action_space.sample())
        for r_ in r.values():
            total_r += r_
        ep_s += 1
        d.update({"total_r": total_r, "episode length": ep_s})
        env.render(text=d)
        if len(env.vehicles) == 0:
            total_r = 0
            print("Reset")
            env.reset()
    env.close()


def _profile():
    import time
    env = MultiAgentRoundaboutEnv({"num_agents": 16})
    obs = env.reset()
    start = time.time()
    for s in range(10000):
        o, r, d, i = env.step(env.action_space.sample())
        if all(d.values()):
            env.reset()
        if (s + 1) % 100 == 0:
            print(
                "Finish {}/10000 simulation steps. Time elapse: {:.4f}. Average FPS: {:.4f}".format(
                    s + 1,
                    time.time() - start, (s + 1) / (time.time() - start)
                )
            )
    print(f"(PGDriveEnvV2) Total Time Elapse: {time.time() - start}")


def _long_run():
    # Please refer to test_ma_roundabout_reward_done_alignment()
    _out_of_road_penalty = 3
    env = MultiAgentRoundaboutEnv(
        {
            "num_agents": 32,
            "vehicle_config": {
                "lidar": {
                    "num_others": 8
                }
            },
            **dict(
                out_of_road_penalty=_out_of_road_penalty,
                crash_vehicle_penalty=1.333,
                crash_object_penalty=11,
                crash_vehicle_cost=13,
                crash_object_cost=17,
                out_of_road_cost=19,
            )
        }
    )
    try:
        obs = env.reset()
        assert env.observation_space.contains(obs)
        for step in range(10000):
            act = env.action_space.sample()
            o, r, d, i = env.step(act)
            if step == 0:
                assert not any(d.values())

            if any(d.values()):
                print("Current Done: {}\nReward: {}".format(d, r))
                for kkk, ddd in d.items():
                    if ddd and kkk != "__all__":
                        print("Info {}: {}\n".format(kkk, i[kkk]))
                print("\n")

            for kkk, rrr in r.items():
                if rrr == -_out_of_road_penalty:
                    assert d[kkk]

            if (step + 1) % 200 == 0:
                print(
                    "{}/{} Agents: {} {}\nO: {}\nR: {}\nD: {}\nI: {}\n\n".format(
                        step + 1, 10000, len(env.vehicles), list(env.vehicles.keys()),
                        {k: (oo.shape, oo.mean(), oo.min(), oo.max())
                         for k, oo in o.items()}, r, d, i
                    )
                )
            if d["__all__"]:
                print('Current step: ', step)
                break
    finally:
        env.close()


if __name__ == "__main__":
    # _draw()
    _vis()
    # _profile()
    # _long_run()
