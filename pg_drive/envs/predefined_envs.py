import os.path as osp
import json
from pg_drive import GeneralizationRacing

root = osp.abspath(osp.dirname(osp.dirname(osp.dirname(__file__))))
assert_path = osp.join(root, "asset", "maps")

from pg_drive.utils.generate_maps import environment_set_dict

import gym
from gym.envs.registration import register


def register_pgdrive():
    for env_name, env_config in environment_set_dict.items():
        map_file = osp.join(assert_path, "{}.json".format(env_name))
        assert osp.exists(map_file)
        env_config["_load_map_from_json"] = map_file

        register(
            id=env_name,
            entry_point=GeneralizationRacing,
            kwargs=dict(config=env_config),
            max_episode_steps=1000,
        )


if __name__ == '__main__':
    register_pgdrive()
    env = gym.make("PGDrive-debug-v0")
    env.reset()
    env.step(env.action_space.sample())
    env.close()
