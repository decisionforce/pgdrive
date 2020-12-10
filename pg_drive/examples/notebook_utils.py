import os.path as osp

path = osp.join(osp.abspath(osp.dirname(__file__)), "ppo_expert", "checkpoint-compressed")


def get_expert(env):
    try:
        import ray
        from ray.rllib.agents.ppo import PPOTrainer
        ray.init(ignore_reinit_error=True)
        expert = PPOTrainer(dict(env="PGDrive-v0", num_workers=0))
        expert.restore(path)

        def func(obs):
            return expert.compute_action(obs)

        return func, True
    except Exception:
        print(
            "Please install ray so that we can load the expert! Please run: pip install ray==1.0.0"
            "\nFailed to load expert, we are now using a random policy."
        )
        return lambda obs: env.action_space.sample(), False
