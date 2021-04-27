import copy
from typing import Union, Any

import numpy as np
from pgdrive.utils.utils import merge_dicts


def merge_config_with_unknown_keys(old_dict, new_dict):
    return merge_config(old_dict, new_dict, new_keys_allowed=True)


def merge_config(old_dict, new_dict, new_keys_allowed=False):
    from pgdrive.utils import PGConfig
    if isinstance(old_dict, PGConfig):
        old_dict = old_dict.get_dict()
    if isinstance(new_dict, PGConfig):
        new_dict = new_dict.get_dict()
    merged = merge_dicts(old_dict, new_dict, allow_new_keys=new_keys_allowed)
    return PGConfig(merged)


def _check_keys(new_config, old_config, prefix=""):
    if isinstance(new_config, PGConfig):
        new_config = new_config.get_dict()
    if isinstance(old_config, PGConfig):
        old_config = new_config.get_dict()
    assert isinstance(new_config, dict)
    assert isinstance(old_config, dict)
    own_keys = set(old_config)
    new_keys = set(new_config)
    if own_keys >= new_keys:
        return True
    else:
        raise KeyError(
            "Unexpected keys: {} in new dict{} when update config. Existing keys: {}.".format(
                new_keys - own_keys, "'s '{}'".format(prefix) if prefix else "", own_keys
            )
        )


def _recursive_check_keys(new_config, old_config, prefix=""):
    _check_keys(new_config, old_config, prefix)
    for k, v in new_config.items():
        new_prefix = prefix + "/" + k if prefix else k
        # if isinstance(v, dict):
        #     _recursive_check_keys(new_config[k], old_config[k], new_prefix)
        if isinstance(v, list):
            for new, old in zip(v, old_config[k]):
                _recursive_check_keys(new, old, new_prefix)


def config_to_dict(config: Union[Any, dict, "PGConfig"], serializable=False) -> dict:
    # Return the flatten and json-able dict
    if not isinstance(config, (dict, PGConfig)):
        return config
    ret = dict()
    for k, v in config.items():
        if isinstance(v, PGConfig):
            v = v.get_dict()
        elif isinstance(v, dict):
            v = {sub_k: config_to_dict(sub_v, serializable) for sub_k, sub_v in v.items()}
        elif serializable and isinstance(v, np.ndarray):
            v = v.tolist()
        ret[k] = v
    return ret


class PGConfig:
    """
    This class aims to check whether user config exists if default config system,
    Mostly, Config is sampled from parameter space in PGDrive

    Besides, the value type will also be checked, but sometimes the value type is not unique (maybe Union[str, int]).
    For these <key, value> items, use PGConfig["your key"] = None to init your PgConfig, then it will not implement
    type check at the first time. key "config" in map.py and key "force_fps" in world.py are good examples.
    """

    def __init__(self, config: Union[dict, "PGConfig"], unchangeable=False):
        self._unchangeable = False
        if isinstance(config, PGConfig):
            config = config.get_dict()
        self._config = self._internal_dict_to_config(copy.deepcopy(config))
        self._types = dict()
        for k, v in self._config.items():
            self._set_item(k, v, allow_overwrite=True)
        self._unchangeable = unchangeable

    def clear(self):
        self._config.clear()

    def register_type(self, key, *types):
        """
        Register special types for item in config. This is used for mixed type declaration.
        Note that is the type is declared as None, then we will not check type for this item.
        """
        assert key in self._config
        self._types[key] = set(types)

    def get_dict(self):
        return config_to_dict(self._config, serializable=False)

    def get_serializable_dict(self):
        return config_to_dict(self._config, serializable=True)

    def update(self, new_dict: Union[dict, "PGConfig"], allow_overwrite=True, stop_recursive_update=None):
        """
        Update this dict with extra configs
        :param new_dict: extra configs
        :param allow_overwrite: whether allowing to add new keys to existing configs or not
        :param stop_recursive_update: Deep update and recursive-check will NOT be applied to keys in stop_recursive_update
        :return: None
        """
        stop_recursive_update = stop_recursive_update or []
        new_dict = new_dict or dict()
        new_dict = copy.deepcopy(new_dict)
        if not allow_overwrite:
            old_keys = set(self._config)
            new_keys = set(new_dict)
            diff = new_keys.difference(old_keys)
            if len(diff) > 0:
                raise KeyError(
                    "'{}' does not exist in existing config. "
                    "Please use config.update(...) to update the config. Existing keys: {}.".format(
                        diff, self._config.keys()
                    )
                )
        for k, v in new_dict.items():
            if k not in self:
                if isinstance(v, dict):
                    v = PGConfig(v)
                self._config[k] = v  # Placeholder
            success = False
            if isinstance(self._config[k], (dict, PGConfig)):
                if k not in stop_recursive_update:
                    success = self._update_dict_item(k, v, allow_overwrite)
                else:
                    self._set_item(k, v, allow_overwrite)
                    success = True
            if not success:
                self._update_single_item(k, v, allow_overwrite)
            if k in self._config and not hasattr(self, k):
                self.__setattr__(k, self._config[k])
        return self

    def _update_dict_item(self, k, v, allow_overwrite):
        if not isinstance(v, (dict, PGConfig)):
            if allow_overwrite:
                return False
            else:
                raise TypeError(
                    "Type error! The item {} has original type {} and updating type {}.".format(
                        k, type(self[k]), type(v)
                    )
                )
        if not isinstance(self[k], PGConfig):
            self._set_item(k, PGConfig(self[k]), allow_overwrite)
        self[k].update(v, allow_overwrite=allow_overwrite)
        return True

    def _update_single_item(self, k, v, allow_overwrite):
        assert not isinstance(v, (dict, PGConfig)), (k, type(v), allow_overwrite)
        self._set_item(k, v, allow_overwrite)

    def items(self):
        return self._config.items()

    def values(self):
        return self._config.values()

    def keys(self):
        return self._config.keys()

    def pop(self, key):
        assert hasattr(self, key)
        self._config.pop(key)
        self.__delattr__(key)

    @classmethod
    def _internal_dict_to_config(cls, d: dict) -> dict:
        ret = dict()
        d = d or dict()
        for k, v in d.items():
            if isinstance(v, dict):
                v = cls(v)
            ret[k] = v
        return ret

    def _check_and_raise_key_error(self, key):
        if key not in self._config:
            raise KeyError(
                "'{}' does not exist in existing config. "
                "Please use config.update(...) to update the config. Existing keys: {}.".format(
                    key, self._config.keys()
                )
            )

    def copy(self, unchangeable=None):
        """If unchangeable is None, then just following the original config's setting."""
        if unchangeable is None:
            unchangeable = self._unchangeable
        return PGConfig(self, unchangeable)

    def __getitem__(self, item):
        self._check_and_raise_key_error(item)
        ret = self._config[item]
        return ret

    def _set_item(self, key, value, allow_overwrite):
        """A helper function to replace __setattr__ and __setitem__!"""
        self._check_and_raise_key_error(key)
        if isinstance(value, np.ndarray) and len(value) == 1:
            # handle 1-d box shape sample
            value = value[0]
            if isinstance(value, (np.float32, np.float64, np.float)):
                value = float(value)
            if isinstance(value, (np.int, np.int32, np.int64, np.uint)):
                value = int(value)
        if self._unchangeable:
            raise ValueError("This config is not changeable!")
        if (not allow_overwrite) and (self._config[key] is not None and value is not None):
            type_correct = isinstance(value, type(self._config[key]))
            if isinstance(self._config[key], PGConfig):
                type_correct = type_correct or isinstance(value, dict)
            if isinstance(self._config[key], float):
                # int can be transformed to float
                type_correct = type_correct or isinstance(value, int)
            if isinstance(self._config[key], int):
                # float can be transformed to int
                type_correct = type_correct or isinstance(value, float)
            if key in self._types:
                if None in self._types[key]:
                    type_correct = True
                type_correct = type_correct or (type(value) in self._types[key])
            assert type_correct, "TypeError: {}:{}".format(key, value)
        self.__setitem__(key, value)

    def __setitem__(self, key, value):
        if self._unchangeable:
            raise ValueError("This config is not changeable!")
        self._config[key] = value
        super(PGConfig, self).__setattr__(key, value)

    def __setattr__(self, key, value):
        if key not in ["_config", "_types", "_unchangeable"]:
            self.__setitem__(key, value)
        else:
            super(PGConfig, self).__setattr__(key, value)

    def __contains__(self, item):
        return item in self._config

    def __repr__(self):
        return str(self._config)

    def __len__(self):
        return len(self._config)

    def __iter__(self):
        return iter(self.keys())

    def check_keys(self, new_dict):
        if isinstance(new_dict, (dict, PGConfig)):
            for k, v in new_dict.items():
                if k not in self:
                    return False, k
                else:
                    return self.check_keys(v)
        else:
            return True, None

    def remove_keys(self, keys):
        for k in keys:
            if k in self:
                self.pop(k)

    def is_identical(self, new_dict: Union[dict, "PGConfig"]) -> bool:
        assert isinstance(new_dict, (dict, PGConfig))
        return _is_identical("", self, "", new_dict)

    def get(self, key, *args):
        return copy.copy(self._config.get(key, *args))

    def force_update(self, new_config):
        self._unchangeable = False
        self.update(new_config)
        self._unchangeable = True

    def force_set(self, key, value):
        self._unchangeable = False
        self[key] = value
        self._unchangeable = True


def _is_identical(k1, v1, k2, v2):
    if k1 != k2:
        return False
    if isinstance(v1, (dict, PGConfig)) or isinstance(v2, (dict, PGConfig)):
        if (not isinstance(v2, (dict, PGConfig))) or (not isinstance(v1, (dict, PGConfig))):
            return False
        if set(v1.keys()) != set(v2.keys()):
            return False
        for k in v1.keys():
            if not _is_identical(k, v1[k], k, v2[k]):
                return False
    else:
        if v1 != v2:
            return False
    return True
