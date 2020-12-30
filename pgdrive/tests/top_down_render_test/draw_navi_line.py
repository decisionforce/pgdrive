from pgdrive import PGDriveEnv
from pgdrive.scene_creator.map import Map, MapGenerateMethod

if __name__ == '__main__':
    env = PGDriveEnv(
        dict(
            environment_num=1,
            map_config={
                Map.GENERATE_METHOD: MapGenerateMethod.BIG_BLOCK_NUM,
                Map.GENERATE_PARA: 30,
            }
        )
    )
    env.reset()
    map = env.current_map.draw_navi_line(env.vehicle, dest_resolution=(2048, 2048), save=True)
