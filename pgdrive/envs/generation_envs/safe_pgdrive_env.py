from pgdrive.envs.pgdrive_env import PGDriveEnv
from pgdrive.pg_config import PGConfig


class SafePGDriveEnv(PGDriveEnv):
    def default_config(self) -> PGConfig:
        extra_config = {
            "accident_prob": 0.5,
            "crash_vehicle_cost": 5,
            "crash_object_cost": 2,
            "crash_vehicle_penalty": 0.,
            "crash_object_penalty": 0.
        }
        config = super(SafePGDriveEnv, self).default_config()
        config.extend_config_with_unknown_keys(extra_config)
        return config

    def custom_info_callback(self):
        self.step_info["cost"] = 0
        if self.step_info["crash_vehicle"]:
            self.step_info["cost"] = self.config["crash_vehicle_cost"]
            self.done = False
        elif self.step_info["crash_object"]:
            self.step_info["cost"] = self.config["crash_object_cost"]
            self.done = False


if __name__ == "__main__":
    env = SafePGDriveEnv(
        {
            "manual_control": True,
            "use_render": True,
            "environment_num": 10,
            "debug": True,
            "cull_scene": True
        }
    )

    o = env.reset()
    total_cost = 0
    for i in range(1, 100000):
        o, r, d, info = env.step([0, 1])
        total_cost += info["cost"]
        env.render(text={"cost": total_cost})
        if d:
            total_cost = 0
            print("Reset")
            env.reset()
    env.close()
