import unittest

from pgdrive.envs.map_gen_env import MapGenEnv


class BasicMapGenTest(unittest.TestCase):
    def setUp(self) -> None:
        self.env = MapGenEnv()

    def test_build(self):
        o = self.env.reset()
        o, r, d, i = self.env.step(self.env.action_space.sample())
        assert self.env.observation_space.contains(o)

    def tearDown(self) -> None:
        self.env.close()


if __name__ == '__main__':
    unittest.main()
