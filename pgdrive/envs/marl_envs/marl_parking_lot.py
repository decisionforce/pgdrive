import gym
import copy
from pgdrive.scene_creator.blocks.parking_lot import ParkingLot
from pgdrive.scene_creator.blocks.straight import Straight
import numpy as np
from pgdrive.envs.multi_agent_pgdrive import MultiAgentPGDrive
from pgdrive.obs import ObservationType
from pgdrive.envs.marl_envs.marl_inout_roundabout import LidarStateObservationMARound
from pgdrive.scene_creator.blocks.first_block import FirstBlock
from pgdrive.scene_creator.blocks.t_intersection import TInterSection
from pgdrive.scene_creator.map import PGMap
from pgdrive.scene_creator.road.road import Road
from pgdrive.utils import get_np_random, norm, PGConfig

MAParkingLotConfig = dict(
    map_config=dict(exit_length=20, lane_num=1),
    top_down_camera_initial_x=80,
    top_down_camera_initial_y=0,
    top_down_camera_initial_z=120
)


class MAParkingLotMap(PGMap):
    def _generate(self, pg_world):
        length = self.config["exit_length"]

        parent_node_path, pg_physics_world = pg_world.worldNP, pg_world.physics_world
        assert len(self.road_network.graph) == 0, "These Map is not empty, please create a new map to read config"

        length = self.config["exit_length"]

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

        last_block = ParkingLot(1, last_block.get_socket(0), self.road_network, 1)
        last_block.construct_block(parent_node_path, pg_physics_world, {"one_side_vehicle_number": 4})
        self.blocks.append(last_block)

        # Build ParkingLot
        TInterSection.EXIT_PART_LENGTH = 10
        last_block = TInterSection(2, last_block.get_socket(index=0), self.road_network, random_seed=1)
        last_block.construct_block(
            parent_node_path,
            pg_physics_world,
            extra_config={
                "t_type":1,
                "change_lane_num":0
                # Note: lane_num is set in config.map_config.lane_num
            }
        )
        self.blocks.append(last_block)


class MultiAgentParkingLotEnv(MultiAgentPGDrive):
    spawn_roads = [
        Road(FirstBlock.NODE_2, FirstBlock.NODE_3),
        -Road(TInterSection.node(2, 0, 0), TInterSection.node(2, 0, 1)),
        -Road(TInterSection.node(2, 2, 0), TInterSection.node(2, 2, 1)),
    ]

    @staticmethod
    def default_config() -> PGConfig:
        return MultiAgentPGDrive.default_config().update(MAParkingLotConfig, allow_overwrite=True)

    def _update_map(self, episode_data: dict = None, force_seed=None):
        if episode_data is not None:
            raise ValueError()
        map_config = self.config["map_config"]
        map_config.update({"seed": self.current_seed})

        if self.current_map is None:
            self.current_seed = 0
            new_map = MAParkingLotMap(self.pg_world, map_config)
            self.maps[self.current_seed] = new_map
            self.current_map = self.maps[self.current_seed]
            spawn_roads = copy.deepcopy(self.spawn_roads)
            spawn_roads += self.current_map.blocks[-2].spawn_roads
            self._spawn_manager.set_spawn_roads(spawn_roads, force_update_all=True)


    def _update_destination_for(self, vehicle):
        # when agent re-joined to the game, call this to set the new route to destination
        return
        end_roads = copy.deepcopy(self.spawn_roads)
        end_roads.remove(vehicle.routing_localization.current_road)
        end_road = -get_np_random(self._DEBUG_RANDOM_SEED).choice(end_roads)  # Use negative road!
        vehicle.routing_localization.set_route(vehicle.lane_index[0], end_road.end_node)

    def get_single_observation(self, vehicle_config: "PGConfig") -> "ObservationType":
        return LidarStateObservationMARound(vehicle_config)


def _draw():
    env = MultiAgentParkingLotEnv()
    o = env.reset()
    from pgdrive.utils import draw_top_down_map
    import matplotlib.pyplot as plt

    plt.imshow(draw_top_down_map(env.current_map))
    plt.show()
    env.close()


def _expert():
    env = MultiAgentParkingLotEnv(
        {
            "vehicle_config": {
                "lidar": {
                    "num_lasers": 240,
                    "num_others": 4,
                    "distance": 50
                },
                "use_saver": True,
                "save_level": 1.
            },
            "pg_world_config": {
                "debug_physics_world": True
            },
            "fast": True,
            # "use_render": True,
            "debug": True,
            "manual_control": True,
            "num_agents": 3,
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
        # env.render(text=d)
        if d["__all__"]:
            print(
                "Finish! Current step {}. Group Reward: {}. Average reward: {}".format(
                    i, total_r, total_r / env.agent_manager.next_agent_count
                )
            )
            break
        if len(env.vehicles) == 0:
            total_r = 0
            print("Reset")
            env.reset()
    env.close()


def _vis_debug_respawn():
    env = MultiAgentParkingLotEnv(
        {
            "horizon": 100000,
            "vehicle_config": {
                "lidar": {
                    "num_lasers": 72,
                    "num_others": 0,
                    "distance": 40
                },
                "show_lidar": False,
            },
            "pg_world_config": {
                "debug_physics_world": True
            },
            "fast": True,
            "use_render": True,
            "debug": False,
            "manual_control": True,
            "num_agents": 3,
        }
    )
    o = env.reset()
    total_r = 0
    ep_s = 0
    for i in range(1, 100000):
        action = {k: [0.0, .0] for k in env.vehicles.keys()}
        o, r, d, info = env.step(action)
        for r_ in r.values():
            total_r += r_
        ep_s += 1
        # d.update({"total_r": total_r, "episode length": ep_s})
        render_text = {
            "total_r": total_r,
            "episode length": ep_s,
            "cam_x": env.main_camera.camera_x,
            "cam_y": env.main_camera.camera_y,
            "cam_z": env.main_camera.top_down_camera_height
        }
        env.render(text=render_text)
        if d["__all__"]:
            print(
                "Finish! Current step {}. Group Reward: {}. Average reward: {}".format(
                    i, total_r, total_r / env.agent_manager.next_agent_count
                )
            )
            # break
        if len(env.vehicles) == 0:
            total_r = 0
            print("Reset")
            env.reset()
    env.close()


def _vis():
    env = MultiAgentParkingLotEnv(
        {
            "horizon": 100000,
            "vehicle_config": {
                "lidar": {
                    "num_lasers": 72,
                    "num_others": 0,
                    "distance": 40
                },
                "show_lidar": False,
            },
            "fast": True,
            "use_render": True,
            "debug": False,
            "manual_control": True,
            "num_agents": 3,
            "delay_done": 1000,
        }
    )
    o = env.reset()
    total_r = 0
    ep_s = 0
    for i in range(1, 100000):
        actions = {k: [1.0, .0] for k in env.vehicles.keys()}
        if len(env.vehicles) == 1:
            actions = {k: [-1.0, .0] for k in env.vehicles.keys()}
        o, r, d, info = env.step(actions)
        for r_ in r.values():
            total_r += r_
        ep_s += 1
        # d.update({"total_r": total_r, "episode length": ep_s})
        render_text = {
            "total_r": total_r,
            "episode length": ep_s,
            "cam_x": env.main_camera.camera_x,
            "cam_y": env.main_camera.camera_y,
            "cam_z": env.main_camera.top_down_camera_height,
            "alive": len(env.vehicles)
        }
        env.render(text=render_text)
        if d["__all__"]:
            print(
                "Finish! Current step {}. Group Reward: {}. Average reward: {}".format(
                    i, total_r, total_r / env.agent_manager.next_agent_count
                )
            )
            env.reset()
            # break
        if len(env.vehicles) == 0:
            total_r = 0
            print("Reset")
            env.reset()
    env.close()


def _profile():
    import time
    env = MultiAgentParkingLotEnv({"num_agents": 3})
    obs = env.reset()
    start = time.time()
    for s in range(10000):
        o, r, d, i = env.step(env.action_space.sample())

        # mask_ratio = env.scene_manager.detector_mask.get_mask_ratio()
        # print("Mask ratio: ", mask_ratio)

        if all(d.values()):
            env.reset()
        if (s + 1) % 100 == 0:
            print(
                "Finish {}/10000 simulation steps. Time elapse: {:.4f}. Average FPS: {:.4f}".format(
                    s + 1,
                    time.time() - start, (s + 1) / (time.time() - start)
                )
            )
    print(f"(MAParkingLot) Total Time Elapse: {time.time() - start}")


def _long_run():
    # Please refer to test_ma_ParkingLot_reward_done_alignment()
    _out_of_road_penalty = 3
    env = MultiAgentParkingLotEnv(
        {
            "num_agents": 3,
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
    # _vis()
    _vis_debug_respawn()
    # _profiwdle()
    # _long_run()