import numpy as np
from gym.spaces import Box
from pgdrive.envs.marl_envs.marl_inout_roundabout import MultiAgentRoundaboutEnv as MARound, \
    LidarStateObservationMARound
from pgdrive.utils import get_np_random


class SVOObs(LidarStateObservationMARound):
    @property
    def observation_space(self):
        space = super(SVOObs, self).observation_space
        assert isinstance(space, Box)
        assert len(space.shape) == 1
        length = space.shape[0] + 1
        space = Box(
            low=np.array([space.low[0]] * length),
            high=np.array([space.high[0]] * length),
            shape=(length,),
            dtype=space.dtype
        )
        return space


class MARoundSVO(MARound):
    def __init__(self, config=None):
        super(MARoundSVO, self).__init__(config)
        self.svo_map = {}

    def get_single_observation(self, vehicle_config):
        return SVOObs(vehicle_config)

    def _get_reset_return(self):
        self.svo_map.clear()
        obses = super(MARoundSVO, self)._get_reset_return()
        ret = {}
        for k, o in obses.items():
            svo, ret[k] = self._add_svo(o)
            self.svo_map[k] = svo
        return ret

    def step(self, actions):
        o, r, d, i = super(MARoundSVO, self).step(actions)
        ret = {}
        for k, v in o.items():
            svo, ret[k] = self._add_svo(v, self.svo_map[k] if k in self.svo_map else None)
            if k not in self.svo_map:
                self.svo_map[k] = svo
            i["svo"] = svo

        # We should also modify reward here!
        pass

        return ret, r, d, i

    def _add_svo(self, o, svo=None):
        svo = get_np_random().uniform(0, 1) if svo is None else svo
        return svo, np.concatenate([o, [svo]])


if __name__ == '__main__':
    env = MARoundSVO()
    o = env.reset()
    assert env.observation_space.contains(o)
    total_r = 0
    ep_s = 0
    for i in range(1, 100000):
        o, r, d, info = env.step({k: [0.0, 1.0] for k in env.vehicles.keys()})
        assert env.observation_space.contains(o)
        for r_ in r.values():
            total_r += r_
        ep_s += 1
        if d["__all__"]:
            print(
                "Finish! Current step {}. Group Reward: {}. Average reward: {}".format(
                    i, total_r, total_r / env.target_vehicle_manager.next_agent_count
                )
            )
            # break
        if len(env.vehicles) == 0:
            total_r = 0
            print("Reset")
            env.reset()
    env.close()
