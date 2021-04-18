import logging

from pgdrive.envs.pgdrive_env_v2 import PGDriveEnvV2
from pgdrive.scene_creator.vehicle.base_vehicle import BaseVehicle
from pgdrive.scene_manager.agent_manager import AgentManager
from pgdrive.utils import setup_logger, PGConfig
from pgdrive.utils.pg_config import merge_dicts
from pgdrive.scene_manager.spawn_manager import SpawnManager

MULTI_AGENT_PGDRIVE_DEFAULT_CONFIG = dict(
    # ===== Multi-agent =====
    is_multi_agent=True,
    num_agents=2,

    # Whether to terminate a vehicle if it crash with others. Since in MA env the crash is extremely dense, so
    # frequently done might not be a good idea.
    crash_done=False,

    # Whether the vehicle can rejoin the episode
    allow_respawn=True,

    # The maximum length of the episode. If allow respawn, then this is the maximum step that respawn can happen. After
    # that, the episode won't terminate until all existing vehicles reach their horizon or done. The vehicle specified
    # horizon is also this value.
    horizon=1000,

    # ===== Vehicle Setting =====
    vehicle_config=dict(lidar=dict(num_lasers=72, distance=40, num_others=0)),
    target_vehicle_configs=dict(),

    # ===== New Reward Setting =====
    out_of_road_penalty=5.0,
    crash_vehicle_penalty=1.0,
    crash_object_penalty=1.0,

    # ===== Environmental Setting =====
    top_down_camera_initial_x=0,
    top_down_camera_initial_y=0,
    top_down_camera_initial_z=120,  # height
    traffic_density=0.,
    auto_termination=False,
    camera_height=4,
)


class MultiAgentPGDrive(PGDriveEnvV2):
    """
    This serve as the base class for Multi-agent PGDrive!
    """

    # A list of road instances denoting which roads afford spawn points. If not set, then search for all
    # possible roads and spawn new agents in them if possible.
    spawn_roads = []

    @staticmethod
    def default_config() -> PGConfig:
        config = PGDriveEnvV2.default_config()
        config.update(MULTI_AGENT_PGDRIVE_DEFAULT_CONFIG)
        return config

    def __init__(self, config=None):
        super(MultiAgentPGDrive, self).__init__(config)
        self.done_observations = dict()
        self._agent_manager = AgentManager(debug=self.config["debug"])


    def _process_extra_config(self, config) -> "PGConfig":
        ret_config = self.default_config().update(
            config, allow_overwrite=False, stop_recursive_update=["target_vehicle_configs"]
        )
        if not ret_config["crash_done"] and ret_config["crash_vehicle_penalty"] > 2:
            logging.warning(
                "Are you sure you wish to set crash_vehicle_penalty={} when crash_done=False?".format(
                    ret_config["crash_vehicle_penalty"]
                )
            )
        if ret_config["use_render"] and ret_config["fast"]:
            logging.warning("Turn fast=False can accelerate Multi-agent rendering performance!")

        self._spawn_manager = SpawnManager(
            spawn_roads=self.spawn_roads,
            exit_length=ret_config["map_config"]["exit_length"],
            lane_num=ret_config["map_config"]["lane_num"],
            num_agents=ret_config["num_agents"],
            vehicle_config=ret_config["vehicle_config"]
        )
        ret_config = self._update_agent_pos_configs(ret_config)
        return ret_config

    def _update_agent_pos_configs(self, config):
        config["target_vehicle_configs"] = self._spawn_manager.get_target_vehicle_configs(
            config["num_agents"], seed=self._DEBUG_RANDOM_SEED
        )
        return config

    def done_function(self, vehicle_id):
        vehicle = self.vehicles[vehicle_id]
        # crash will not done
        done, done_info = super(MultiAgentPGDrive, self).done_function(vehicle_id)
        if vehicle.crash_vehicle and not self.config["crash_done"]:
            assert done_info["crash_vehicle"] or done_info["arrive_dest"] or done_info["out_of_road"]
            if not (done_info["arrive_dest"] or done_info["out_of_road"]):
                # Does not revert done if high-priority termination happens!
                done = False
        elif vehicle.out_of_route and vehicle.on_lane and not vehicle.crash_sidewalk:
            pass  # Do nothing when out of the road!! This is not the SAFETY environment!
        return done, done_info

    def step(self, actions):
        self._update_spaces_if_needed()
        o, r, d, i = super(MultiAgentPGDrive, self).step(actions)
        o, r, d, i = self._after_vehicle_done(o, r, d, i)

        # Update respawn manager
        if self.episode_steps >= self.config["horizon"]:
            self._agent_manager.set_allow_respawn(False)
        self._spawn_manager.update(self.vehicles, self.current_map)
        new_obs_dict = self._respawn()
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

    def reset(self, *args, **kwargs):
        for v in self.done_vehicles.values():
            v.chassis_np.node().setStatic(False)

        # Multi-agent related reset
        # avoid create new observation!
        obses = list(self.done_observations.values()) + list(self.observations.values())
        assert len(obses) == len(self.config["target_vehicle_configs"].keys())
        self.observations = {k: v for k, v in zip(self.config["target_vehicle_configs"].keys(), obses)}
        self.done_observations = dict()

        self.observation_space = self._get_observation_space()
        self.action_space = self._get_action_space()

        return super(MultiAgentPGDrive, self).reset(*args, **kwargs)

    def _reset_vehicles(self):
        # TODO(pzh) deprecated this function in future!
        vehicles = list(self.vehicles.values()) + list(self.done_vehicles.values())
        assert len(vehicles) == len(self.observations)
        self.vehicles = {k: v for k, v in zip(self.observations.keys(), vehicles)}
        self.done_vehicles = {}
        self.for_each_vehicle(lambda v: v.reset(self.current_map))

    def _after_vehicle_done(self, obs=None, reward=None, dones: dict = None, info=None):
        for id, done in dones.items():
            if done and id in self.vehicles.keys():
                v = self.vehicles.pop(id)
                v.prepare_step([0, -1])
                self.done_vehicles[id] = v
        for v in self.done_vehicles.values():
            if v.speed < 1:
                v.chassis_np.node().setStatic(True)
        return obs, reward, dones, info

    def _get_vehicles(self):
        return {
            name: BaseVehicle(self.pg_world, self._get_target_vehicle_config(new_config))
            for name, new_config in self.config["target_vehicle_configs"].items()
        }

    def _get_observations(self):
        return {
            name: self.get_single_observation(self._get_target_vehicle_config(new_config))
            for name, new_config in self.config["target_vehicle_configs"].items()
        }

    def _get_target_vehicle_config(self, extra_config: dict):
        """
        Newly introduce method
        """
        vehicle_config = merge_dicts(self.config["vehicle_config"], extra_config, allow_new_keys=False)
        return PGConfig(vehicle_config)

    def _update_spaces_if_needed(self):
        assert self.is_multi_agent
        current_obs_keys = set(self.observations.keys())
        for k in current_obs_keys:
            if k not in self.vehicles:
                o = self.observations.pop(k)
                self.done_observations[k] = o
        current_obs_keys = set(self.observation_space.spaces.keys())
        for k in current_obs_keys:
            if k not in self.vehicles:
                self.observation_space.spaces.pop(k)
                # self.action_space.spaces.pop(k)  # Action space is updated in _respawn

    def _after_lazy_init(self):
        super(MultiAgentPGDrive, self)._after_lazy_init()

        # Use top-down view by default
        if hasattr(self, "main_camera") and self.main_camera is not None:
            top_down_camera_height = self.config["top_down_camera_initial_z"]
            self.main_camera.camera.setPos(0, 0, top_down_camera_height)
            self.main_camera.top_down_camera_height = top_down_camera_height
            self.main_camera.stop_chase(self.pg_world)
            self.main_camera.camera_x += self.config["top_down_camera_initial_x"]
            self.main_camera.camera_y += self.config["top_down_camera_initial_y"]


if __name__ == "__main__":
    from pgdrive.scene_creator.blocks.first_block import FirstBlock

    setup_logger(True)
    env = MultiAgentPGDrive(
        {
            "use_render": True,
            "debug": False,
            "manual_control": True,
            "pg_world_config": {
                "pstats": False
            },
            "target_vehicle_configs": {
                "agent0": {
                    "spawn_longitude": 10,
                    "spawn_lateral": 1.5,
                    "spawn_lane_index": (FirstBlock.NODE_1, FirstBlock.NODE_2, 1),
                    # "show_lidar": True
                    "show_side_detector": True
                },
                "agent1": {
                    "spawn_longitude": 10,
                    # "show_lidar": True,
                    "spawn_lateral": -1,
                    "spawn_lane_index": (FirstBlock.NODE_1, FirstBlock.NODE_2, 0),
                },
            }
        }
    )
    o = env.reset()
    total_r = 0
    for i in range(1, 100000):
        o, r, d, info = env.step(env.action_space.sample())
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
