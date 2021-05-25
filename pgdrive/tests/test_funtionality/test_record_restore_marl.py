import json

from pgdrive.envs.marl_envs.marl_inout_roundabout import MultiAgentRoundaboutEnv
from pgdrive.scene_creator.map import Map, MapGenerateMethod
from pgdrive.scene_manager.traffic_manager import TrafficMode
from pgdrive.utils import setup_logger


def test_save_episode(vis=True):
    setup_logger(True)

    test_dump = False

    env = MultiAgentRoundaboutEnv(dict(use_render=vis, manual_control=vis, record_episode=True, horizon=300))
    try:
        o = env.reset()
        epi_info = None
        for i in range(1, 100000 if vis else 2000):
            o, r, d, info = env.step({agent_id: [0, .2] for agent_id in env.vehicles.keys()})
            if vis:
                env.render()
            if d["__all__"]:
                epi_info = env.scene_manager.dump_episode()

                # test dump json
                if test_dump:
                    with open("test.json", "w") as f:
                        json.dump(epi_info, f)
                break

        o = env.reset(epi_info)
        for i in range(1, 100000 if vis else 2000):
            o, r, d, info = env.step({agent_id: [0, 0.1] for agent_id in env.vehicles.keys()})
            if vis:
                env.render()
            if d["__all__"]:
                break
    finally:
        env.close()


if __name__ == "__main__":
    test_save_episode(vis=True)
