"""
Please install ray before running this script:
    pip install ray==1.0.0
"""
from pg_drive import GeneralizationRacing
from pg_drive.examples import get_expert

expert = get_expert()
env = GeneralizationRacing(dict(use_render=True, environment_num=10000))
obs = env.reset()
try:
    while True:
        action = expert(obs)
        obs, reward, done, info = env.step(action)
        frame = env.render("rgb_array")
        if done:
            obs = env.reset()
finally:
    print("Closing the environment!")
    env.close()
