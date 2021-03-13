from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.scene_creator.ego_vehicle.base_vehicle import BaseVehicle
from pgdrive.utils import setup_logger

setup_logger(debug=True)


class TestEnv(PGDriveEnv):
    def __init__(self):
        super(TestEnv, self).__init__(
            {
                "image_source": "depth_cam",
                "manual_control": True,
                "use_render": False,
                "use_image": True
            }
        )

    def reset(self):
        if self.vehicle is not None:
            self.for_each_vehicle(lambda v: v.destroy(self.pg_world))
            self.vehicles = {
                a: BaseVehicle(self.pg_world, env.config["vehicle_config"])
                for a in self.multi_agent_action_space.keys()
            }

            self.for_each_vehicle(self.add_modules_for_vehicle)
            if self.main_camera is not None:
                self.main_camera.reset(self.vehicle, env.pg_world)
        super(TestEnv, self).reset()


if __name__ == "__main__":
    env = TestEnv()

    env.reset()
    for i in range(1, 100000):
        o, r, d, info = env.step([0, 1])
        # env.render("Test: {}".format(i))
        if d:
            env.reset()
    env.close()
