import os.path as osp

path = osp.join(osp.abspath(osp.dirname(__file__)), "ppo_expert", "checkpoint-compressed")


def get_expert():
    import ray
    from ray.rllib.agents.ppo import PPOTrainer

    ray.init(ignore_reinit_error=True)
    expert = PPOTrainer(dict(env="PGDrive-v0", num_workers=0))
    expert.restore(path)

    def func(obs):
        return expert.compute_action(obs)

    return func
