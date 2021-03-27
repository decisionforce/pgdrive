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


def config_to_dict(config: Union[Any, dict, "PGConfig"]) -> dict:
    # Return the flatten and json-able dict
    if not isinstance(config, (dict, PGConfig)):
        return config
    ret = dict()
    for k, v in config.items():
        if isinstance(v, PGConfig):
            v = v.get_dict()
        elif isinstance(v, dict):
            v = {sub_k: config_to_dict(sub_v) for sub_k, sub_v in v.items()}
        elif isinstance(v, np.ndarray):
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

    def __init__(self, config: Union[dict, "PGConfig"]):
        if isinstance(config, PGConfig):
            config = config.get_dict()
        self._config = self._internal_dict_to_config(copy.deepcopy(config))
        self._types = dict()
        for k, v in self._config.items():
            setattr(self, k, v)

    def clear(self):
        self._config.clear()

    def add(self, key, value):
        raise ValueError("Deprecated!")
        assert key not in self._config, "KeyError: {} exists in config".format(key)
        if isinstance(value, (dict, PGConfig)):
            value = PGConfig(value)
        self.__setitem__(key, value)

    def register_type(self, key, *types):
        """
        Register special types for item in config. This is used for mixed type declaration.
        Note that is the type is declared as None, then we will not check type for this item.
        """
        assert key in self._config
        self._types[key] = set(types)

    def get_dict(self):
        return config_to_dict(self._config)

    def update(self, new_dict: Union[dict, "PGConfig"], allow_overwrite=False):
        new_dict = new_dict or dict()
        new_dict = copy.deepcopy(new_dict)
        for k, v in new_dict.items():
            if k not in self:
                if not allow_overwrite:
                    self._check_and_raise_key_error(k)
                else:
                    self._config[k] = v  # Placeholder
            success = False
            if isinstance(self._config[k], (dict, PGConfig)):
                success = self._update_dict_item(k, v, allow_overwrite)
            if not success:
                self._update_single_item(k, v, allow_overwrite)
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
        self[k].update(v, allow_overwrite=allow_overwrite)
        return True

    def _update_single_item(self, k, v, allow_overwrite):
        assert not isinstance(v, (dict, PGConfig)), (k, type(v), allow_overwrite)
        if allow_overwrite:
            self._config[k] = v
        setattr(self, k, v)

    def items(self):
        return self._config.items()

    def values(self):
        return self._config.values()

    def keys(self):
        return self._config.keys()

    def pop(self, key):
        self._config.pop(key)

    @classmethod
    def _internal_dict_to_config(cls, d: dict) -> dict:
        ret = dict()
        for k, v in d.items():
            if isinstance(v, dict):
                v = cls(v)
            ret[k] = v
        return ret

    def extend_config_with_unknown_keys(self, extra_config: dict) -> None:
        raise ValueError(
            "This function is deprecated. Please explicitly use pgdrive.utils.merge_config or merge_"
            "config_with_unknown_keys."
        )

    def _check_and_raise_key_error(self, key):
        if key not in self._config:
            raise KeyError(
                "'{}' does not exist in existing config. "
                "Please use config.update(..., allow_overwrite=True) to update the config. Existing keys: {}."
                    .format(key, self._config.keys())
            )

    def copy(self):
        return copy.deepcopy(self)

    def __getitem__(self, item):
        self._check_and_raise_key_error(item)
        ret = self._config[item]
        if isinstance(ret, np.ndarray) and len(ret) == 1:
            # handle 1-d box shape sample
            ret = ret[0]
        return ret

    def __setitem__(self, key, value):
        self._check_and_raise_key_error(key)
        if self._config[key] is not None and value is not None:
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

        self._config[key] = value

        super(PGConfig, self).__setattr__(key, value)

    def __setattr__(self, key, value):
        if key not in ["_config", "_types"]:
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
