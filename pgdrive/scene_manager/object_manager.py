from pgdrive.scene_creator.blocks.curve import Curve
from pgdrive.scene_creator.blocks.ramp import InRampOnStraight, OutRampOnStraight
from pgdrive.scene_creator.lane.abs_lane import AbstractLane
from pgdrive.scene_creator.map import Map
from pgdrive.scene_creator.object.traffic_object import Object
from pgdrive.scene_creator.road.road import Road
from pgdrive.scene_creator.road.road_network import LaneIndex
from pgdrive.utils import RandomEngine
from pgdrive.world.pg_world import PGWorld


class ObjectsManager(RandomEngine):
    """
    This class is used to manager all static object, such as traffic cones, warning tripod.
    """

    # the distance between break-down vehicle and alert
    ALERT_DIST = 10

    # distance between two cones
    ACCIDENT_AREA_LEN = 12
    CONE_LONGITUDE = 2
    CONE_LATERAL = 1

    def __init__(self):
        self._spawned_objects = []

        # init random engine
        super(ObjectsManager, self).__init__()

    def clear_objects(self, pg_world: PGWorld):
        """
        Clear all objects in th scene
        """
        for obj in self._spawned_objects:
            obj.destroy(pg_world=pg_world)
        self._spawned_objects = []

    def spawn_one_object(
            self,
            object_type: str,
            lane: AbstractLane,
            lane_index: LaneIndex,
            longitude: float,
            lateral: float,
            static: bool = False
    ) -> Object:
        """
        Spawn an object by assigning its type and position on the lane
        :param object_type: object name or the class name of the object
        :param lane: object will be spawned on this lane
        :param lane_index:the lane index of the spawn point
        :param longitude: longitude position on this lane
        :param lateral: lateral position on  this lane
        :param static: static object can not be moved by any force
        :return: None
        """
        for t in Object.type():
            if t.__name__ == object_type or t.NAME == object_type:
                obj = t.make_on_lane(lane, lane_index, longitude, lateral)
                obj.set_static(static)
                self._spawned_objects.append(obj)
                return obj
        raise ValueError("No object named {}, so it can not be spawned".format(object_type))

    def generate(self, scene_mgr, pg_world: PGWorld, map: Map, accident_prob: float):
        """
        Generate an accident scene or construction scene on block
        :param scene_mgr: SceneManager used to access other managers
        :param pg_world: PGWorld class
        :param map: Map class, containing road network and blocks
        :param accident_prob: Objects will be generated on each block with this probability
        :return: None
        """
        self.update_random_seed(map.random_seed)
        if abs(accident_prob - 0.0) < 1e-2:
            return
        for block in map.blocks:
            if type(block) not in [Curve, InRampOnStraight, OutRampOnStraight]:
                # blocks with exists do not generate accident scene
                continue
            if self.np_random.rand() > accident_prob:
                # prob filter
                continue

            # generate scene
            block.PROHIBIT_TRAFFIC_GENERATION = True
            road_1 = Road(block.pre_block_socket.positive_road.end_node, block.road_node(0, 0))
            road_2 = Road(block.road_node(0, 0), block.road_node(0, 1))
            accident_road = self.np_random.choice([road_1, road_2])

            is_ramp = isinstance(block, InRampOnStraight) or isinstance(block, OutRampOnStraight)
            on_left = True if self.np_random.rand() > 0.5 or (accident_road is road_2 and is_ramp) else False
            accident_lane_idx = 0 if on_left else -1

            longitude = self.np_random.rand() * accident_road.get_lanes(scene_mgr.map.road_network)[
                accident_lane_idx].length - self.ACCIDENT_AREA_LEN / self.CONE_LONGITUDE
            lateral_len = scene_mgr.map.lane_width * ((scene_mgr.map.lane_num - 2) * self.np_random.rand() + 1)
            self.accident_scene(scene_mgr, pg_world, accident_road.lane_index(accident_lane_idx), longitude,
                                lateral_len, on_left)

    def break_down_scene(self, scene_mgr, pg_world: PGWorld, lane_index: LaneIndex, longitude: float):
        lane = scene_mgr.map.road_network.get_lane(lane_index)

        breakdown_vehicle = scene_mgr.traffic_mgr.spawn_one_vehicle(
            scene_mgr.traffic_mgr.random_vehicle_type(), lane, longitude, False)
        breakdown_vehicle.attach_to_pg_world(pg_world.pbr_worldNP, pg_world.physics_world)

        alert = self.spawn_one_object("Traffic Triangle", lane, lane_index, longitude - self.ALERT_DIST, 0)
        alert.attach_to_pg_world(pg_world.pbr_worldNP, pg_world.physics_world)

    def accident_scene(self, scene_mgr, pg_world: PGWorld, lane_index: LaneIndex,
                       longitude_position: float,
                       lateral_len: float,
                       on_left=False):
        """
        Generate an accident scene on the most left or most right lane
        :param scene_mgr: SceneManager class
        :param pg_world: PGWorld class
        :param lane_index: lane index used to find the lane in map
        :param longitude_position: longitude position of the accident on the lane
        :param lateral_len: the distance that traffic cones extend on lateral direction
        :param on_left: on left or right side
        :return: None
        """
        lane = scene_mgr.map.road_network.get_lane(lane_index)
        lat_num = int(lateral_len / self.CONE_LATERAL)
        longitude_num = int(self.ACCIDENT_AREA_LEN / self.CONE_LONGITUDE)
        lat_1 = [lat * self.CONE_LATERAL for lat in range(lat_num)]
        lat_2 = [lat_num * self.CONE_LATERAL] * (longitude_num + 1)
        lat_3 = [(lat_num - lat - 1) * self.CONE_LATERAL for lat in range(int(lat_num))]

        total_long_num = lat_num * 2 + longitude_num + 1
        pos = [(long * self.CONE_LONGITUDE, lat - lane.width / 2) for long, lat in
               zip(range(total_long_num), lat_1 + lat_2 + lat_3)]
        left = 1 if on_left else -1
        longitude_position = min(longitude_position, lane.length - total_long_num * self.CONE_LONGITUDE)
        for p in pos:
            p_ = (p[0] + longitude_position, left * p[1])
            cone = self.spawn_one_object("Traffic Cone", lane, lane_index, *p_)
            cone.attach_to_pg_world(pg_world.pbr_worldNP, pg_world.physics_world)

        # v_pos = [7, 9]
        # for v_long in v_pos:
        #     v = scene_mgr.traffic_mgr.spawn_one_vehicle(
        #         scene_mgr.traffic_mgr.random_vehicle_type(), lane, v_long, False
        #     )
        #     v.attach_to_pg_world(pg_world.pbr_worldNP, pg_world.physics_world)
