from pgdrive.scene_creator.blocks.curve import Curve
from pgdrive.utils.engine_utils import get_pgdrive_engine
from pgdrive.scene_creator.blocks.ramp import InRampOnStraight, OutRampOnStraight
from pgdrive.scene_creator.blocks.straight import Straight
from pgdrive.scene_creator.lane.abs_lane import AbstractLane
from pgdrive.scene_creator.map.map import Map
from pgdrive.scene_creator.object.traffic_object import TrafficObject
from pgdrive.scene_creator.object.base_object import BaseObject
from pgdrive.scene_creator.road.road import Road
from pgdrive.scene_creator.road.road_network import LaneIndex
from pgdrive.utils import RandomEngine


class ObjectManager(RandomEngine):
    """
    This class is used to manager all static object, such as traffic cones, warning tripod.
    """

    # the distance between break-down vehicle and alert
    ALERT_DIST = 10

    # accident scene setting
    ACCIDENT_AREA_LEN = 10
    ACCIDENT_LANE_MIN_LEN = 50

    # distance between two cones
    CONE_LONGITUDE = 2
    CONE_LATERAL = 1
    PROHIBIT_SCENE_PROB = 0.5  # the reset is the probability of break_down_scene

    def __init__(self):
        self._spawned_objects = []
        self._block_objects = []
        self.accident_prob = 0.

        # init random engine
        super(ObjectManager, self).__init__()

    def reset(self, map: Map, accident_prob: float = 0):
        """
        Clear all objects in th scene
        """
        self._clear_objects()
        self.update_random_seed(map.random_seed)
        self.accident_prob = accident_prob
        for block in map.blocks:
            block.construct_block_buildings(self)

    def _clear_objects(self):
        # only destroy self-generated objects
        for obj in self._spawned_objects:
            obj.destroy()
        self._spawned_objects = []
        for obj in self._block_objects:
            obj.node_path.detachNode()
        self._block_objects = []

    def add_block_buildings(self, building: BaseObject, render_node):
        self._block_objects.append(building)
        building.node_path.reparentTo(render_node)

    def spawn_one_object(
        self,
        object_type: str,
        lane: AbstractLane,
        lane_index: LaneIndex,
        longitude: float,
        lateral: float,
        static: bool = False
    ) -> TrafficObject:
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
        for t in TrafficObject.type():
            if t.__name__ == object_type or t.NAME == object_type:
                obj = t.make_on_lane(lane, lane_index, longitude, lateral)
                obj.set_static(static)
                self._spawned_objects.append(obj)
                return obj
        raise ValueError("No object named {}, so it can not be spawned".format(object_type))

    def generate(self):
        """
        Generate an accident scene or construction scene on block
        :return: None
        """
        engine = get_pgdrive_engine()
        accident_prob = self.accident_prob
        if abs(accident_prob - 0.0) < 1e-2:
            return
        for block in engine.map.blocks:
            if type(block) not in [Straight, Curve, InRampOnStraight, OutRampOnStraight]:
                # blocks with exists do not generate accident scene
                continue
            if self.np_random.rand() > accident_prob:
                # prob filter
                continue

            road_1 = Road(block.pre_block_socket.positive_road.end_node, block.road_node(0, 0))
            road_2 = Road(block.road_node(0, 0), block.road_node(0, 1)) if not isinstance(block, Straight) else None
            accident_road = self.np_random.choice([road_1, road_2]) if not isinstance(block, Curve) else road_2
            accident_road = road_1 if accident_road is None else accident_road
            is_ramp = isinstance(block, InRampOnStraight) or isinstance(block, OutRampOnStraight)
            on_left = True if self.np_random.rand() > 0.5 or (accident_road is road_2 and is_ramp) else False
            accident_lane_idx = 0 if on_left else -1

            _debug = engine.world_config["_debug_crash_object"]
            if _debug:
                on_left = True

            lane = accident_road.get_lanes(engine.map.road_network)[accident_lane_idx]
            longitude = lane.length - self.ACCIDENT_AREA_LEN

            if lane.length < self.ACCIDENT_LANE_MIN_LEN:
                continue

            # generate scene
            block.PROHIBIT_TRAFFIC_GENERATION = True

            # TODO(pzh) This might not worked in MARL envs when the route is also has changeable lanes.
            lateral_len = engine.map.config[engine.map.LANE_WIDTH]

            lane = engine.map.road_network.get_lane(accident_road.lane_index(accident_lane_idx))
            if self.np_random.rand() > 0.5 or _debug:
                self.prohibit_scene(lane, accident_road.lane_index(accident_lane_idx), longitude, lateral_len, on_left)
            else:
                accident_lane_idx = self.np_random.randint(engine.map.config[engine.map.LANE_NUM])
                self.break_down_scene(lane, accident_road.lane_index(accident_lane_idx), longitude)

    def break_down_scene(self, lane: AbstractLane, lane_index: LaneIndex, longitude: float):
        engine = get_pgdrive_engine()
        breakdown_vehicle = engine.traffic_manager.spawn_one_vehicle(
            engine.traffic_manager.random_vehicle_type(), lane, longitude, False
        )
        breakdown_vehicle.attach_to_pg_world(engine.pbr_worldNP, engine.physics_world)
        breakdown_vehicle.set_break_down()

        alert = self.spawn_one_object("Traffic Triangle", lane, lane_index, longitude - self.ALERT_DIST, 0)
        alert.attach_to_pg_world(engine.pbr_worldNP, engine.physics_world)

    def prohibit_scene(
        self, lane: AbstractLane, lane_index: LaneIndex, longitude_position: float, lateral_len: float, on_left=False
    ):
        """
        Generate an accident scene on the most left or most right lane
        :param lane lane object
        :param lane_index: lane index used to find the lane in map
        :param longitude_position: longitude position of the accident on the lane
        :param lateral_len: the distance that traffic cones extend on lateral direction
        :param on_left: on left or right side
        :return: None
        """
        engine = get_pgdrive_engine()
        lat_num = int(lateral_len / self.CONE_LATERAL)
        longitude_num = int(self.ACCIDENT_AREA_LEN / self.CONE_LONGITUDE)
        lat_1 = [lat * self.CONE_LATERAL for lat in range(lat_num)]
        lat_2 = [lat_num * self.CONE_LATERAL] * (longitude_num + 1)
        lat_3 = [(lat_num - lat - 1) * self.CONE_LATERAL for lat in range(int(lat_num))]

        total_long_num = lat_num * 2 + longitude_num + 1
        pos = [
            (long * self.CONE_LONGITUDE, lat - lane.width / 2)
            for long, lat in zip(range(-int(total_long_num / 2), int(total_long_num / 2)), lat_1 + lat_2 + lat_3)
        ]
        left = 1 if on_left else -1
        for p in pos:
            p_ = (p[0] + longitude_position, left * p[1])
            cone = self.spawn_one_object("Traffic Cone", lane, lane_index, *p_)
            cone.attach_to_pg_world(engine.pbr_worldNP, engine.physics_world)
            # TODO refactor traffic and traffic system to make it compatible

    def destroy(self):
        self._clear_objects()
        self._spawned_objects = None
        self._block_objects = None

    @property
    def objects(self):
        return self._spawned_objects + self._block_objects
