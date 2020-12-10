"""
Please install ray before running this script:
    pip install ray==1.0.0
"""
from pg_drive import GeneralizationRacing
from pg_drive.examples import get_expert

env = GeneralizationRacing(dict(use_render=True, environment_num=10000))
expert, success = get_expert(env)
obs = env.reset()
try:
    while True:
        action = expert(obs)
        obs, reward, done, info = env.step(action)
        frame = env.render("rgb_array", text=None if success else "Failed to load agent.\nRunning random policy now!")
        if done:
            obs = env.reset()
finally:
    print("Closing the environment!")
    env.close()
