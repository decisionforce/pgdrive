import json

from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.component.map.base_map import BaseMap, MapGenerateMethod
from pgdrive.manager.traffic_manager import TrafficMode
from pgdrive.utils import setup_logger


class TestEnv(PGDriveEnv):
    def __init__(self, save_episode=True, vis=True):
        super(TestEnv, self).__init__(
            {
                "environment_num": 1,
                "traffic_density": 0.1,
                "start_seed": 5,
                # "manual_control": vis,
                "use_render": vis,
                "traffic_mode": TrafficMode.Trigger,
                "record_episode": save_episode,
                "map_config": {
                    BaseMap.GENERATE_TYPE: MapGenerateMethod.BIG_BLOCK_SEQUENCE,
                    BaseMap.GENERATE_CONFIG: "XTXTXTXTXT",
                    BaseMap.LANE_WIDTH: 3.5,
                    BaseMap.LANE_NUM: 3,
                }
            }
        )


def test_save_episode(vis=False):
    setup_logger(True)

    test_dump = True

    env = TestEnv(vis=vis)
    try:
        o = env.reset()
        epi_info = None
        for i in range(1, 100000 if vis else 2000):
            o, r, d, info = env.step([0, 1])
            if vis:
                env.render()
            if d:
                epi_info = env.pgdrive_engine.dump_episode()

                # test dump json
                if test_dump:
                    with open("test.json", "w") as f:
                        json.dump(epi_info, f)
                break

        o = env.reset(epi_info)
        for i in range(1, 100000 if vis else 2000):
            o, r, d, info = env.step([0, 1])
            if vis:
                env.render()
            if d:
                break
    finally:
        env.close()


if __name__ == "__main__":
    test_save_episode(vis=True)
