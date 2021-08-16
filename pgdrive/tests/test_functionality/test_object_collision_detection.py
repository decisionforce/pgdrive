from pgdrive.component.vehicle.vehicle_type import LVehicle
from pgdrive.component.static_object.traffic_object import TrafficCone, TrafficTriangle
from pgdrive.constants import BodyName, TerminationState, DEFAULT_AGENT
from pgdrive.envs import PGDriveEnv


class TestEnv(PGDriveEnv):
    """
    now for test use and demo use only
    """
    @classmethod
    def default_config(cls):
        config = super(TestEnv, cls).default_config()
        config.update(
            {
                "environment_num": 1,
                "traffic_density": 0.1,
                "start_seed": 5,
                # "traffic_mode":"respawn",
                "debug_physics_world": False,
                "debug": False,
                "map": "CSRCR"
            }
        )
        return config

    def __init__(self, config=None):
        super(TestEnv, self).__init__(config)
        self.breakdown_vehicle = None
        self.alert = None

    def reset(self, episode_data: dict = None, force_seed=None):
        ret = super(TestEnv, self).reset(episode_data)
        self.vehicle.max_speed = 60
        lane = self.current_map.road_network.graph[">>>"]["1C0_0_"][0]
        self.breakdown_vehicle = self.engine.spawn_object(
            self.engine.traffic_manager.random_vehicle_type(),
            vehicle_config={
                "spawn_lane_index": lane.index,
                "spawn_longitude": 30
            }
        )

        lane_ = self.current_map.road_network.graph[">>>"]["1C0_0_"][1]
        breakdown_vehicle = self.engine.spawn_object(
            LVehicle, vehicle_config={
                "spawn_lane_index": lane_.index,
                "spawn_longitude": 30
            }
        )

        self.alert = self.engine.spawn_object(TrafficTriangle, lane=lane, longitude=22, lateral=0)

        # part 1
        lane = self.current_map.road_network.graph["1C0_1_"]["2S0_0_"][2]
        pos = [
            (-20, lane.width / 3), (-15.6, lane.width / 4), (-12.1, 0), (-8.7, -lane.width / 4),
            (-4.2, -lane.width / 2), (-0.7, -lane.width * 3 / 4), (4.1, -lane.width), (7.3, -lane.width),
            (11.5, -lane.width), (15.5, -lane.width), (20.0, -lane.width), (23.2, -lane.width), (29.1, -lane.width),
            (32.9, -lane.width / 2), (37.0, 0), (40.0, lane.width / 2)
        ]

        for p in pos:
            cone = self.engine.spawn_object(TrafficCone, lane=lane, longitude=p[0], lateral=p[1])
        from pgdrive.component.vehicle.vehicle_type import SVehicle, XLVehicle
        v_pos = [8, 14]
        v_type = [SVehicle, XLVehicle]
        for v_long, v_t in zip(v_pos, v_type):
            v = self.engine.spawn_object(
                v_t, vehicle_config={
                    "spawn_lane_index": lane.index,
                    "spawn_longitude": v_long
                }
            )

        # part 2
        lane = self.current_map.road_network.graph["3R0_0_"]["3R0_1_"][0]
        pos = [
            (-20, lane.width / 3), (-15.6, lane.width / 4), (-12.1, 0), (-8.7, -lane.width / 4),
            (-4.2, -lane.width / 2), (-0.7, -lane.width * 3 / 4), (4.1, -lane.width), (7.3, -lane.width),
            (11.5, -lane.width), (15.5, -lane.width), (20.0, -lane.width), (23.2, -lane.width), (29.1, -lane.width),
            (32.9, -lane.width / 2), (37.0, 0), (40.0, lane.width / 2)
        ]

        for p in pos:
            p_ = (p[0] + 5, -p[1])
            cone = self.engine.spawn_object(TrafficCone, lane=lane, longitude=p_[0], lateral=p_[1])

        v_pos = [14, 19]
        for v_long in v_pos:
            v = self.engine.spawn_object(
                self.engine.traffic_manager.random_vehicle_type(),
                vehicle_config={
                    "spawn_lane_index": lane.index,
                    "spawn_longitude": v_long
                }
            )

        alert = self.engine.spawn_object(TrafficTriangle, lane=lane, longitude=-35, lateral=0)

        alert = self.engine.spawn_object(TrafficTriangle, lane=lane, longitude=-60, lateral=0)

        # part 3
        lane = self.current_map.road_network.graph["4C0_0_"]["4C0_1_"][2]
        pos = [
            (-12.1, 0), (-8.7, -lane.width / 4), (-4.2, -lane.width / 2), (-0.7, -lane.width * 3 / 4),
            (4.1, -lane.width), (7.3, -lane.width), (11.5, -lane.width), (15.5, -lane.width), (20.0, -lane.width),
            (23.2, -lane.width), (29.1, -lane.width), (32.9, -lane.width / 2), (37.0, 0), (40.0, lane.width / 2)
        ]

        for p in pos:
            p_ = (p[0] + 5, p[1] * 3.5 / 3)
            cone = self.engine.spawn_object(TrafficCone, lane=lane, longitude=p_[0], lateral=p_[1])

        v_pos = [14, 19]
        for v_long in v_pos:
            v = self.engine.spawn_object(
                self.engine.traffic_manager.random_vehicle_type(),
                vehicle_config={
                    "spawn_lane_index": lane.index,
                    "spawn_longitude": v_long
                }
            )

        # part 4
        lane = self.current_map.road_network.graph["4C0_1_"]["5R0_0_"][0]
        pos = [(-12, lane.width / 4), (-8.1, 0), (-4, -lane.width / 4), (-0.1, -lane.width / 2), (4, -lane.width)]

        for p in pos:
            p_ = (p[0] + 60, -p[1] * 3.5 / 3)
            cone = self.engine.spawn_object(TrafficCone, lane=lane, longitude=p_[0], lateral=p_[1])

        return ret


def test_object_collision_detection(render=False):
    env = TestEnv(
        {
            "manual_control": render,
            "use_render": render,
            "debug": False,
            "vehicle_config": {
                "show_lidar": True
            }
        }
    )
    try:
        o = env.reset()
        lane_index = (">>", ">>>", 0)
        alert = env.engine.spawn_object(
            TrafficTriangle, lane=env.current_map.road_network.get_lane(lane_index), longitude=22, lateral=0
        )
        lane_index = (">>", ">>>", 2)
        alert = env.engine.spawn_object(
            TrafficCone, lane=env.current_map.road_network.get_lane(lane_index), longitude=22, lateral=0
        )
        crash_obj = False
        detect_obj = False
        for i in range(1, 100000 if render else 2000):
            o, r, d, info = env.step([0, 1])
            for obj in env.observations[DEFAULT_AGENT].detected_objects:
                if obj.getNode().hasPythonTag(BodyName.Traffic_cone):
                    detect_obj = True
            if render:
                env.render()
            if info[TerminationState.CRASH_OBJECT]:
                crash_obj = True
                break
        assert crash_obj and detect_obj, "Can not crash with object!"
    finally:
        env.close()


if __name__ == "__main__":
    test_object_collision_detection(render=True)
