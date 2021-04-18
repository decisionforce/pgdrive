from collections import defaultdict
from math import cos, sin

import numpy as np
from gym.spaces import Box
from pgdrive.envs.marl_envs.marl_inout_roundabout import MultiAgentRoundaboutEnv as MARound, \
    LidarStateObservationMARound
from pgdrive.utils import get_np_random, norm


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
            shape=(length, ),
            dtype=space.dtype
        )
        return space


class MARoundSVO(MARound):
    @classmethod
    def default_config(cls):
        config = super(MARoundSVO, cls).default_config()
        config.update(
            dict(
                # Number of near vehicles that participates in reward computing
                num_neighbours=4,

                # Two mode to compute utility for each vehicle:
                # "linear": util = r_me * svo + r_other * (1 - svo), svo in [0, 1]
                # "angle": util = r_me * cos(svo) + r_other * sin(svo), svo in [0, pi/2]
                svo_mode="linear",
            )
        )
        return config

    def __init__(self, config=None):
        super(MARoundSVO, self).__init__(config)
        self.svo_map = {}
        self.distance_map = defaultdict(lambda: defaultdict(lambda: float("inf")))
        assert self.config["svo_mode"] in ["linear", "angle"]

    def get_single_observation(self, vehicle_config):
        return SVOObs(vehicle_config)

    def _get_reset_return(self):
        self.svo_map.clear()
        self._update_distance_map()
        obses = super(MARoundSVO, self)._get_reset_return()
        ret = {}
        for k, o in obses.items():
            svo, ret[k] = self._add_svo(o)
            self.svo_map[k] = svo
        return ret

    def step(self, actions):
        o, r, d, i = super(MARoundSVO, self).step(actions)
        self._update_distance_map()

        ret = {}
        for k, v in o.items():
            svo, ret[k] = self._add_svo(v, self.svo_map[k] if k in self.svo_map else None)
            if k not in self.svo_map:
                self.svo_map[k] = svo
            if i[k]:
                i[k]["svo"] = svo

        # We should also modify reward here!
        new_rewards = {}
        K = self.config["num_neighbours"]
        if K >= 1:
            for k, own_r in r.items():
                dist_to_others = self.distance_map[k]
                dist_to_others_list = sorted(dist_to_others, key=lambda k: dist_to_others[k])

                other_rewards = []
                for other_k in dist_to_others_list[:K]:
                    other_rewards.append(r[other_k])
                if len(other_rewards) == 0:
                    other_reward = own_r
                else:
                    other_reward = np.mean(other_rewards)

                if self.config["svo_mode"] == "linear":
                    new_r = self.svo_map[k] * own_r + (1 - self.svo_map[k]) * other_reward
                elif self.config["svo_mode"] == "angle":
                    svo = self.svo_map[k] * np.pi / 2
                    new_r = cos(svo) * own_r + sin(svo) * other_reward

                new_rewards[k] = new_r
        else:
            new_rewards = r

        return ret, new_rewards, d, i

    def _add_svo(self, o, svo=None):
        svo = get_np_random().uniform(0, 1) if svo is None else svo
        return svo, np.concatenate([o, [svo]])

    def _find_k_nearest(self, v_id):
        pass

    def _update_distance_map(self):
        self.distance_map.clear()
        keys = list(self.vehicles.keys())
        for c1 in range(0, len(keys) - 1):
            for c2 in range(c1 + 1, len(keys)):
                k1 = keys[c1]
                k2 = keys[c2]
                v1 = self.vehicles[k1]
                v2 = self.vehicles[k2]
                p1 = v1.position
                p2 = v2.position
                distance = norm(p1[0] - p2[0], p1[1] - p2[1])
                self.distance_map[k1][k2] = distance
                self.distance_map[k2][k1] = distance


if __name__ == '__main__':
    env = MARoundSVO({"num_agents": 8, "num_neighbours": 0})
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
