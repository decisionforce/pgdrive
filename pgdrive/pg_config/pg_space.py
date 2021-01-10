import typing as tp
from collections import namedtuple

from pgdrive.pg_config.space import Dict, Box, Discrete

PgBoxSpace = namedtuple("PgBoxSpace", "max min")
PgDiscreteSpace = namedtuple("PgDiscreteSpace", "number")
PgConstantSpace = namedtuple("PgConstantSpace", "value")


class PgSpace(Dict):
    """
    length = PgSpace(name="length",max=50.0,min=10.0)
    Usage:
    PgSpace({"lane_length":length})
    """
    def __init__(self, our_config: tp.Dict[str, tp.Union[PgBoxSpace, PgDiscreteSpace, PgConstantSpace]]):
        super(PgSpace, self).__init__(PgSpace.wrap2gym_space(our_config))
        self.parameters = set(our_config.keys())

    @staticmethod
    def wrap2gym_space(our_config):
        ret = dict()
        for key, value in our_config.items():
            if isinstance(value, PgBoxSpace):
                ret[key] = Box(low=value.min, high=value.max, shape=(1, ))
            elif isinstance(value, PgDiscreteSpace):
                ret[key] = Discrete(value.number)
            elif isinstance(value, PgConstantSpace):
                ret[key] = Box(low=value.value, high=value.value, shape=(1, ))
            else:
                raise ValueError("{} can not be wrapped in gym space".format(key))
        return ret
