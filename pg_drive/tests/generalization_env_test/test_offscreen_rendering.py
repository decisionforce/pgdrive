from pg_drive.envs.generalization_racing import GeneralizationRacing
from pg_drive.utils import setup_logger

setup_logger(debug=True)

if __name__ == "__main__":
    shape = (84, 84)
    env = GeneralizationRacing({
        "traffic_density": 0.0,
        "use_render": False,
        "use_rgb": True,
        "vehicle_config": dict(front_cam=shape)
    })

    env.reset()
    for i in range(10):
        o, r, d, info = env.step(env.action_space.sample())
        assert o["image"].shape == shape + (3,), (o["image"].shape, shape)
        if d:
            env.reset()
    env.close()

    shape = (200, 88)
    env = GeneralizationRacing({
        "traffic_density": 0.0,
        "use_render": False,
        "use_rgb": True,
        "vehicle_config": dict(front_cam=shape)
    })
    env.reset()
    for i in range(10):
        o, r, d, info = env.step(env.action_space.sample())
        assert o["image"].shape == shape + (3,), (o["image"].shape, shape)
        if d:
            env.reset()
    env.close()
