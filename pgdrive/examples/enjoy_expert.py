"""
Please feel free to run this script to enjoy a journey carrying out by a professional driver!
Our expert can drive in 10000 maps with almost 90% likelihood to achieve the destination.

Note: This script require rendering, please following the installation instruction to setup a proper
environment that allows popping up an window.
"""
from pgdrive import PGDriveEnv
from pgdrive.examples import expert, get_terminal_state

env = PGDriveEnv(dict(use_render=True, environment_num=10000))
obs = env.reset()
ep_reward = 0
ep_len = 0
try:
    while True:
        action = expert(obs)
        obs, reward, done, info = env.step(action)
        ep_reward += reward
        ep_len += 1
        # env.render()
        if done:
            print("Episode terminated! Length: {}, Reward: {:.4f}, Terminal state: {}.".format(
                ep_len, ep_reward, get_terminal_state(info)
            ))
            ep_reward = 0
            ep_len = 0
            obs = env.reset()
finally:
    print("Closing the environment!")
    env.close()
