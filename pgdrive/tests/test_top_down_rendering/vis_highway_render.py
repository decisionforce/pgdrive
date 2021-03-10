import time

from pgdrive.envs.top_down_env import TopDownSingleFramePGDriveEnv
from pgdrive.utils import setup_logger


def vis_top_down_render_with_panda_render():
    setup_logger(True)

    env = TopDownSingleFramePGDriveEnv(
        {
            "environment_num": 1,
            "manual_control": True,
            "use_render": True,
            "use_image": False,
            "traffic_mode": "reborn"
        }
    )
    o = env.reset()
    s = time.time()
    for i in range(1, 100000):
        o, r, d, info = env.step(env.action_space.sample())
        env.render(
            text={
                "vehicle_num": len(env.scene_manager.traffic_mgr.vehicles),
                "traffic_vehicle": len(env.scene_manager.traffic_mgr.traffic_vehicles)
            }
        )
        # if d:
        #     env.reset()
        if i % 1000 == 0:
            print("Steps: {}, Time: {}".format(i, time.time() - s))
    env.close()


if __name__ == '__main__':
    vis_top_down_render_with_panda_render()
