"""
Please install ray before running this script:
    pip install ray==1.0.0
"""
import os.path as osp

import ray
from pg_drive import GeneralizationRacing
from ray.rllib.agents.ppo import PPOTrainer

path = osp.join(osp.abspath(osp.dirname(__file__)), "ppo_expert", "checkpoint-compressed")

ray.init(ignore_reinit_error=True)
expert = PPOTrainer(dict(env="PGDrive-v0", num_workers=0))
expert.restore(path)

env = GeneralizationRacing(dict(use_render=True, environment_num=100))

obs = env.reset()
try:
    while True:
        action = expert.compute_action(obs)
        obs, reward, done, info = env.step(action)
        frame = env.render("rgb_array")
        if done:
            obs = env.reset()
finally:
    print("Closing the environment!")
    env.close()
