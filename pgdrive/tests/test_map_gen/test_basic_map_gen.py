import unittest

import numpy as np

from pgdrive.envs.map_gen_env import MapGenEnv, parameter_space_to_gym_space
from pgdrive.pg_config.pg_space import PgSpace, PgBoxSpace, PgDiscreteSpace


class BasicMapGenTest(unittest.TestCase):
    def setUp(self) -> None:
        self.env = MapGenEnv()

    # def test_build(self):
    #     o = self.env.reset()
    #     o, r, d, i = self.env.step(self.env.action_space.sample())
    #     assert self.env.observation_space.contains(o)

    def test_parameter_space_to_gym_space(self):
        config = PgSpace({
            "aaa": PgBoxSpace(min=40.0, max=80.0),
            "bbb": PgDiscreteSpace(2),
            "ccc": PgBoxSpace(min=-10000.0, max=10000.0)
        })
        act_space, mappings, transform = parameter_space_to_gym_space(config)
        assert len(np.unique(act_space.low)) == 1
        assert len(np.unique(act_space.high)) == 1
        assert act_space.low[0] == -1
        assert act_space.high[0] == 1

        def _assert_para(para):
            assert "aaa" in para
            assert "bbb" in para
            assert "ccc" in para
            assert 40 <= para["aaa"] <= 80
            assert -10000 <= para["ccc"] <= 10000
            assert para["bbb"] in [0, 1]

        for _ in range(10):
            act = act_space.sample()
            para = transform(act)
            _assert_para(para)

        for act_x in [-1, 0, 0.5, 1]:
            for act_y in [-1, 0, 0.5, 1]:
                for act_z in [-1, 0, 0.5, 1]:
                    para = transform([act_x, act_y, act_z])
                    _assert_para(para)

        print(f"Success fully tested the action to parameter mechanism. The parameter is: {para}")

    def tearDown(self) -> None:
        self.env.close()


if __name__ == '__main__':
    unittest.main()
