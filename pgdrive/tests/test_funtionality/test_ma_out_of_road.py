import numpy as np
from pgdrive.envs.marl_envs.marl_inout_roundabout import MultiAgentRoundaboutEnv


def test_vehicle_immediate_respawn():
    # out of road
    last_pos = {}
    env = MultiAgentRoundaboutEnv({
        "horizon": 500, "num_agents": 40, "out_of_road_penalty": 777, "crash_done": False,
    })
    env._DEBUG_RANDOM_SEED = 1
    env.seed(env._DEBUG_RANDOM_SEED)
    try:
        obs = env.reset()
        assert env.observation_space.contains(obs)
        for action in [-1, 1]:
            for step in range(5000):
                for k, v in env.vehicles.items():
                    last_pos[v.name] = (v.position, k)
                act = {k: [action, 1] for k in env.vehicles.keys()}
                o, r, d, i = env.step(act)
                for kkk, ddd in d.items():
                    if ddd and kkk != "__all__":
                        if kkk not in env._agent_manager.pending_objects:
                            v_id = env._agent_manager._AgentManager__agents_finished_this_frame[kkk]
                            v = env._agent_manager.get_object(v_id)
                        else:
                            v = env._agent_manager.pending_objects[kkk]
                        if not np.all(np.array(v.position) != np.array(last_pos[v.name][0])):
                            print('111')
                        assert np.all(np.array(v.position) != np.array(last_pos[v.name][0])), \
                            (v.position, last_pos[v.name], kkk)

                env.render(mode="top_down")

                if d["__all__"]:
                    env.reset()
                    break
    finally:
        env.close()
        env._DEBUG_RANDOM_SEED = None


if __name__ == '__main__':
    test_vehicle_immediate_respawn()
