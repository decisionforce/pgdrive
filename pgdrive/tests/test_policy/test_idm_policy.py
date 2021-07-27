import copy

from pgdrive.envs.base_env import BASE_DEFAULT_CONFIG
from pgdrive.policy.idm_policy import IDMPolicy
from pgdrive.scene_creator.vehicle.base_vehicle import BaseVehicle


def _create_vehicle():
    config = copy.deepcopy(BASE_DEFAULT_CONFIG)["vehicle_config"]
    return BaseVehicle(config, random_seed=0)


def test_idm_policy():
    v = _create_vehicle()
    policy = IDMPolicy()
    action = policy.act(v)


if __name__ == '__main__':
    test_idm_policy()
