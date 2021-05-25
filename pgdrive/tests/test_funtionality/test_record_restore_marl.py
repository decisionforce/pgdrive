import json

from pgdrive.envs.marl_envs.marl_inout_roundabout import MultiAgentRoundaboutEnv
from pgdrive.scene_creator.map import Map, MapGenerateMethod
from pgdrive.scene_manager.traffic_manager import TrafficMode
from pgdrive.utils import setup_logger


def test_save_episode(vis=False):
    """
    1. Set record_episode=True to record each episode
    2. dump_episode when done[__all__] == True
    3. You can keep recent episodes
    4. Input episode data to reset() function can replay the episode !
    """

    setup_logger(True)

    test_dump = True
    dump_recent_episode = 5
    dump_count = 0
    env = MultiAgentRoundaboutEnv(dict(use_render=vis, manual_control=vis, record_episode=True, horizon=100))
    try:
        # Test Record
        o = env.reset()
        epi_info = None
        for i in range(1, 100000 if vis else 600):
            o, r, d, info = env.step({agent_id: [0, .2] for agent_id in env.vehicles.keys()})
            if vis:
                env.render()
            if d["__all__"]:
                epi_info = env.scene_manager.dump_episode()
                # test dump json
                if test_dump:
                    with open("test_dump_{}.json".format(dump_count), "w") as f:
                        json.dump(epi_info, f)
                    dump_count += 1
                    dump_count = dump_count % dump_recent_episode
                env.reset()

        # input episode_info to restore
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
    test_save_episode(vis=False)
