import copy
import logging

from pgdrive.component.blocks.first_block import FirstPGBlock
from pgdrive.component.road.road import Road
from pgdrive.constants import TerminationState
from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.manager.spawn_manager import SpawnManager
from pgdrive.utils import setup_logger, get_np_random, Config
from pgdrive.utils.config import merge_dicts

MULTI_AGENT_PGDRIVE_DEFAULT_CONFIG = dict(
    # ===== Multi-agent =====
    is_multi_agent=True,
    num_agents=15,
    # If num_agents is set to None, then endless vehicles will be added only the empty spawn points exist
    random_agent_model=False,

    # Whether to terminate a vehicle if it crash with others. Since in MA env the crash is extremely dense, so
    # frequently done might not be a good idea.
    crash_done=True,
    out_of_road_done=True,
    delay_done=25,  # Wait for 5 seconds in real world.

    # Whether the vehicle can rejoin the episode
    allow_respawn=True,
    spawn_roads=[Road(FirstPGBlock.NODE_2, FirstPGBlock.NODE_3)],

    # The maximum length of the episode. If allow respawn, then this is the maximum step that respawn can happen. After
    # that, the episode won't terminate until all existing vehicles reach their horizon or done. The vehicle specified
    # horizon is also this value.
    horizon=1000,

    # Use to determine what neighborhood means
    neighbours_distance=10,

    # ===== Vehicle Setting =====
    vehicle_config=dict(lidar=dict(num_lasers=72, distance=40, num_others=0), random_color=True, not_randomize=False),
    target_vehicle_configs=dict(),

    # ===== New Reward Setting =====
    out_of_road_penalty=10,
    crash_vehicle_penalty=10,
    crash_object_penalty=10,
    crash_vehicle_cost=1,
    crash_object_cost=1,
    out_of_road_cost=0,  # Do not count out of road into cost!

    # ===== Environmental Setting =====
    traffic_density=0.,
    auto_termination=False,
    camera_height=4,
    load_map_from_json=False,
    _load_map_from_json=""
)


class MultiAgentPGDrive(PGDriveEnv):
    """
    This serve as the base class for Multi-agent PGDrive!
    """

    # A list of road instances denoting which roads afford spawn points. If not set, then search for all
    # possible roads and spawn new agents in them if possible.

    @staticmethod
    def default_config() -> Config:
        config = PGDriveEnv.default_config()
        config.update(MULTI_AGENT_PGDRIVE_DEFAULT_CONFIG)
        return config

    def _merge_extra_config(self, config) -> "Config":
        ret_config = self.default_config().update(
            config, allow_add_new_key=False, stop_recursive_update=["target_vehicle_configs"]
        )
        if not ret_config["crash_done"] and ret_config["crash_vehicle_penalty"] > 2:
            logging.warning(
                "Are you sure you wish to set crash_vehicle_penalty={} when crash_done=False?".format(
                    ret_config["crash_vehicle_penalty"]
                )
            )
        if ret_config["use_render"] and ret_config["fast"]:
            logging.warning("Turn fast=False can accelerate Multi-agent rendering performance!")

        if "prefer_track_agent" in config and config["prefer_track_agent"]:
            ret_config["target_vehicle_configs"][config["prefer_track_agent"]]["am_i_the_special_one"] = True
        ret_config["vehicle_config"]["random_agent_model"] = ret_config["random_agent_model"]
        return ret_config

    def _post_process_config(self, config):
        from pgdrive.manager.spawn_manager import SpawnManager
        config = super(MultiAgentPGDrive, self)._post_process_config(config)
        ret_config = config
        # merge basic vehicle config into target vehicle config
        target_vehicle_configs = dict()
        num_agents = ret_config["num_agents"] if ret_config["num_agents"] != -1 else SpawnManager.max_capacity(
            config["spawn_roads"], config["map_config"]["exit_length"], config["map_config"]["lane_num"]
        )
        for id in range(num_agents):
            agent_id = "agent{}".format(id)
            config = ret_config["vehicle_config"]
            if agent_id in ret_config["target_vehicle_configs"]:
                config.update(ret_config["target_vehicle_configs"][agent_id])
                config["not_randomize"] = True
            target_vehicle_configs[agent_id] = config
        ret_config["target_vehicle_configs"] = target_vehicle_configs
        return ret_config

    def done_function(self, vehicle_id):
        done, done_info = super(MultiAgentPGDrive, self).done_function(vehicle_id)
        if done_info[TerminationState.CRASH] and (not self.config["crash_done"]):
            assert done_info[TerminationState.CRASH_VEHICLE] or \
                   done_info[TerminationState.SUCCESS] or done_info[TerminationState.OUT_OF_ROAD]
            if not (done_info[TerminationState.SUCCESS] or done_info[TerminationState.OUT_OF_ROAD]):
                # Does not revert done if high-priority termination happens!
                done = False

        if done_info[TerminationState.OUT_OF_ROAD] and (not self.config["out_of_road_done"]):
            assert done_info[TerminationState.CRASH_VEHICLE] or \
                   done_info[TerminationState.SUCCESS] or done_info[TerminationState.OUT_OF_ROAD]
            if not done_info[TerminationState.SUCCESS]:
                done = False

        return done, done_info

    def step(self, actions):
        o, r, d, i = super(MultiAgentPGDrive, self).step(actions)
        o, r, d, i = self._after_vehicle_done(o, r, d, i)

        # Update respawn manager
        if self.episode_steps >= self.config["horizon"] or self.engine.replay_system is not None:
            self.agent_manager.set_allow_respawn(False)
        new_obs_dict = self._respawn_vehicles(randomize_position=self.config["random_traffic"])
        if new_obs_dict:
            for new_id, new_obs in new_obs_dict.items():
                o[new_id] = new_obs
                r[new_id] = 0.0
                i[new_id] = {}
                d[new_id] = False

        # Update __all__
        d["__all__"] = (
                ((self.episode_steps >= self.config["horizon"]) and (all(d.values()))) or (len(self.vehicles) == 0)
                or (self.episode_steps >= 5 * self.config["horizon"])
        )
        if d["__all__"]:
            for k in d.keys():
                d[k] = True

        return o, r, d, i

    def _after_vehicle_done(self, obs=None, reward=None, dones: dict = None, info=None):
        if self.engine.replay_system is not None:
            return obs, reward, dones, info
        for v_id, v_info in info.items():
            if v_info.get("episode_length", 0) >= self.config["horizon"]:
                if dones[v_id] is not None:
                    info[v_id][TerminationState.MAX_STEP] = True
                    dones[v_id] = True
                    self.dones[v_id] = True
        for dead_vehicle_id, done in dones.items():
            if done:
                self.agent_manager.finish(
                    dead_vehicle_id, ignore_delay_done=info[dead_vehicle_id].get(TerminationState.SUCCESS, False)
                )
                self._update_camera_after_finish()
        return obs, reward, dones, info

    def _update_camera_after_finish(self):
        if self.main_camera is not None and self.current_track_vehicle.id not in self.engine.agent_manager._active_objects \
                and self.engine.task_manager.hasTaskNamed(self.main_camera.CHASE_TASK_NAME):
            self.chase_camera()

    def _get_observations(self):
        return {
            name: self.get_single_observation(new_config)
            for name, new_config in self.config["target_vehicle_configs"].items()
        }

    def _respawn_vehicles(self, randomize_position=False):
        new_obs_dict = {}
        if not self.agent_manager.allow_respawn:
            return new_obs_dict
        while True:
            new_id, new_obs = self._respawn_single_vehicle(randomize_position=randomize_position)
            if new_obs is not None:
                new_obs_dict[new_id] = new_obs
            else:
                break
        return new_obs_dict

    def _respawn_single_vehicle(self, randomize_position=False):
        """
        Arbitrary insert a new vehicle to a new spawn place if possible.
        """
        safe_places_dict = self.engine.spawn_manager.get_available_respawn_places(
            self.current_map, randomize=randomize_position
        )
        if len(safe_places_dict) == 0:
            return None, None
        born_place_index = get_np_random(self._DEBUG_RANDOM_SEED).choice(list(safe_places_dict.keys()), 1)[0]
        new_spawn_place = safe_places_dict[born_place_index]

        new_agent_id, vehicle = self.agent_manager.propose_new_vehicle()
        new_spawn_place_config = new_spawn_place["config"]
        new_spawn_place_config = self.engine.spawn_manager.update_destination_for(new_agent_id, new_spawn_place_config)
        vehicle.config.update(new_spawn_place_config)
        vehicle.reset()
        vehicle.after_step()
        self.dones[new_agent_id] = False  # Put it in the internal dead-tracking dict.

        new_obs = self.observations[new_agent_id].observe(vehicle)
        return new_agent_id, new_obs

    def setup_engine(self):
        super(MultiAgentPGDrive, self).setup_engine()
        self.engine.register_manager("spawn_manager", SpawnManager())


def _test():
    setup_logger(True)
    env = MultiAgentPGDrive(
        {
            "num_agents": 12,
            "allow_respawn": False,
            "use_render": True,
            "debug": False,
            "fast": True,
            "manual_control": True,
        }
    )
    o = env.reset()
    total_r = 0
    for i in range(1, 100000):
        # o, r, d, info = env.step(env.action_space.sample())
        o, r, d, info = env.step({v_id: [0, 1] for v_id in env.vehicles.keys()})
        for r_ in r.values():
            total_r += r_
        # o, r, d, info = env.step([0,1])
        d.update({"total_r": total_r})
        # env.render(text=d)
        env.render(mode="top_down")
        if len(env.vehicles) == 0:
            total_r = 0
            print("Reset")
            env.reset()
    env.close()


def _vis():
    setup_logger(True)
    env = MultiAgentPGDrive(
        {
            "use_render": True,
            "fast": True,
            "num_agents": 5,
            "start_seed": 8000,
            "environment_num": 1,
            "map": "SSS",

            # "allow_respawn": False,
            # "manual_control": True,
        }
    )
    o = env.reset()
    total_r = 0
    for i in range(1, 100000):
        # o, r, d, info = env.step(env.action_space.sample())
        o, r, d, info = env.step({v_id: [0.0, 0.0] for v_id in env.vehicles.keys()})
        for r_ in r.values():
            total_r += r_
        # o, r, d, info = env.step([0,1])
        # d.update({"total_r": total_r})
        env.render(mode="top_down")
        # env.reset()
        if len(env.vehicles) == 0:
            total_r = 0
            print("Reset")
            env.reset()
    env.close()


def pygame_replay(name, env_class, save=False, other_traj=None, film_size=(1000, 1000), extra_config={}):
    import copy
    import json
    import pygame
    extra_config["use_render"] = True
    env = env_class(extra_config)
    ckpt = "metasvodist_{}_best.json".format(name) if other_traj is None else other_traj
    with open(ckpt, "r") as f:
        traj = json.load(f)
    o = env.reset(copy.deepcopy(traj))
    env.main_camera.set_follow_lane(True)
    frame_count = 0
    while True:
        o, r, d, i = env.step(env.action_space.sample())
        env.engine.force_fps.toggle()
        env.render(mode="top_down", num_stack=50, film_size=film_size, history_smooth=0)
        if save:
            pygame.image.save(env._top_down_renderer._runtime, "{}_{}.png".format(name, frame_count))
        frame_count += 1
        if len(env.engine.replay_system.restore_episode_info) == 0:
            env.close()


def panda_replay(name, env_class, save=False, other_traj=None, extra_config={}):
    import copy
    import json
    import pygame
    extra_config.update({"use_render": True})
    env = env_class(extra_config)
    ckpt = "metasvodist_{}_best.json".format(name) if other_traj is None else other_traj
    with open(ckpt, "r") as f:
        traj = json.load(f)
    o = env.reset(copy.deepcopy(traj))
    env.main_camera.set_follow_lane(True)
    frame_count = 0
    while True:
        o, r, d, i = env.step(env.action_space.sample())
        env.engine.force_fps.toggle()
        if save:
            pygame.image.save(env._top_down_renderer._runtime, "{}_{}.png".format(name, frame_count))
        frame_count += 1
        if len(env.engine.replay_system.restore_episode_info) == 0:
            env.close()


if __name__ == '__main__':
    # _test()
    _vis()
