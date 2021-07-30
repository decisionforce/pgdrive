from pgdrive import PGDriveEnv
from pgdrive.component.map.base_map import BaseMap, MapGenerateMethod
from pgdrive.utils.draw_top_down_map import draw_top_down_map

if __name__ == '__main__':
    env = PGDriveEnv(
        dict(
            environment_num=1,
            map_config={
                BaseMap.GENERATE_TYPE: MapGenerateMethod.BIG_BLOCK_SEQUENCE,
                BaseMap.GENERATE_CONFIG: "OCrRCTXRCCCCrOr",
                BaseMap.LANE_WIDTH: 3.5,
                BaseMap.LANE_NUM: 3,
            }
        )
    )
    for i in range(100):
        env.reset()
        map = draw_top_down_map(env.current_map)
        print("Finish {} maps!".format(i + 1))
