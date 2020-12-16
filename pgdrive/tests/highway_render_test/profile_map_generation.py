from pgdrive import PGDriveEnv

env = PGDriveEnv(dict(environment_num=100))
for i in range(100):
    env.reset()
    map = env.get_map()
    print("Finish {} maps!".format(i))
