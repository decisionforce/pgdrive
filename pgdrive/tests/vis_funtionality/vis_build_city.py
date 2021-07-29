import matplotlib.pyplot as plt

from pgdrive import PGDriveEnv
from pgdrive.component.map.city_map import CityMap
from pgdrive.utils import draw_top_down_map
from pgdrive.utils.engine_utils import initialize_pgdrive_engine, close_pgdrive_engine, set_global_random_seed


def _t(num_blocks):
    default_config = PGDriveEnv.default_config()
    default_config["pg_world_config"].update({"use_render": False, "use_image": False, "debug": False})
    initialize_pgdrive_engine(default_config, None)
    set_global_random_seed(0)
    try:
        map_config = default_config["map_config"]
        map_config.update(dict(type="block_num", config=num_blocks))
        map = CityMap(map_config)
        fig = draw_top_down_map(map, (1024, 1024))
        plt.imshow(fig, cmap="bone")
        plt.xticks([])
        plt.yticks([])
        plt.title("Building a City with {} blocks!".format(num_blocks))
        plt.show()
    finally:
        close_pgdrive_engine()


if __name__ == "__main__":
    _t(20)
