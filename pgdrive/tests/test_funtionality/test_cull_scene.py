from pgdrive.envs.multi_agent_pgdrive import MultiAgentPGDrive
from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.utils import PGConfig


def test_cull_scene(use_render=False):
    class TestCull(MultiAgentPGDrive):
        def default_config(self) -> PGConfig:
            config = MultiAgentPGDrive.default_config()
            config.update({
                "target_vehicle_configs": {},
                "num_agents": 0,
                "crash_done": True,
            }, allow_overwrite=True)
            return config

    for _ in range(5):
        env = TestCull(
            {
                "use_render": use_render,
                "manual_control": False,
                "fast": use_render,
                "map": "SSSSCS",
                "debug": True,
                "target_vehicle_configs": {
                    "agent0": {
                        "spawn_longitude": 10,
                        "spawn_lateral": 2.5,
                        "spawn_lane_index": ("5C0_0_", "5C0_1_", 1),
                    },
                    "agent1": {
                        "spawn_longitude": 12,  # locate a little forward
                        "spawn_lateral": 2.2,
                        "spawn_lane_index": ("5C0_0_", "5C0_1_", 1),
                    }
                },
                "num_agents": 2,
                "traffic_density": 0.4,
            }
        )
        try:
            pass_test = False
            o = env.reset()
            for i in range(1, 200):
                actions = {"agent0": [1, 0.2], "agent1": [0, 0]}
                if "agent0" not in env.vehicles:
                    actions.pop("agent0")
                if "agent1" not in env.vehicles:
                    actions.pop("agent1")
                o, r, d, info = env.step(actions)
                if any(d.values()):
                    if info["agent0"]["crash_vehicle"]:
                        pass_test = True
                    break
            assert pass_test, "Cull scene error! collision function is invalid!"
        finally:
            env.close()


if __name__ == "__main__":
    # test_cull_scene(True)
    test_cull_scene(False)
