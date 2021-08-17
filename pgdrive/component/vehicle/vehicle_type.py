from pgdrive.component.vehicle.base_vehicle import BaseVehicle

factor = 1


class DefaultVehicle(BaseVehicle):
    LENGTH = 4.51
    WIDTH = 1.852
    HEIGHT = 1.19
    TIRE_RADIUS = 0.313
    MASS = 1300
    LATERAL_TIRE_TO_CENTER = 0.815
    FRONT_WHEELBASE = 1.05234
    REAR_WHEELBASE = 1.4166
    path = [['ferra/scene.gltf', (factor, factor, factor), (0, 0.0, 0.), 0]]


class XLVehicle(BaseVehicle):
    LENGTH = 5.8
    WIDTH = 2.3
    HEIGHT = 2.8
    TIRE_RADIUS = 0.37
    REAR_WHEELBASE = 1.075
    FRONT_WHEELBASE = 1.726
    LATERAL_TIRE_TO_CENTER = 0.831
    CHASSIS_TO_WHEEL_AXIS = 0.3
    MASS = 1600
    path = [['new/truck/scene.gltf', (factor, factor, factor), (0, 0.3, 0.04), 0]]


class LVehicle(BaseVehicle):
    LENGTH = 4.5
    WIDTH = 1.86
    HEIGHT = 1.85
    TIRE_RADIUS = 0.39
    REAR_WHEELBASE = 1.10751
    FRONT_WHEELBASE = 1.391
    LATERAL_TIRE_TO_CENTER = 0.75
    MASS = 1300
    path = [
        ['new/lada/scene.gltf', (factor, factor, factor), (0, -0.25, 0.07), 0],
    ]


class MVehicle(BaseVehicle):
    LENGTH = 4.4
    WIDTH = 1.85
    HEIGHT = 1.37
    TIRE_RADIUS = 0.39
    REAR_WHEELBASE = 1.203
    FRONT_WHEELBASE = 1.285
    LATERAL_TIRE_TO_CENTER = 0.803
    MASS = 1200

    path = [
        ['new/130/scene.gltf', (factor, factor, factor), (0, -0.05, 0.07), 0],
    ]


class SVehicle(BaseVehicle):
    LENGTH = 4.25
    WIDTH = 1.7
    HEIGHT = 1.7
    LATERAL_TIRE_TO_CENTER = 0.7
    FRONT_WHEELBASE = 1.4126
    REAR_WHEELBASE = 1.07
    TIRE_RADIUS = 0.376
    MASS = 800

    path = [
        ['new/beetle/scene.gltf', (factor, factor, factor), (0, -0.2, 0.03), 0],
    ]


vehicle_type = {"s": SVehicle, "m": MVehicle, "l": LVehicle, "xl": XLVehicle, "default": DefaultVehicle}
only_default_agent = [0, 0, 0, 0, 1]


def random_vehicle_type(np_random, p=None):
    prob = [1 / len(vehicle_type) for _ in range(len(vehicle_type))] if p is None else p
    return vehicle_type[np_random.choice(list(vehicle_type.keys()), p=prob)]
