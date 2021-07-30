from typing import Dict

from pgdrive.component.base_class.configurable import Configurable
from pgdrive.component.base_class.nameable import Nameable
from pgdrive.component.base_class.randomizable import Randomizable
from pgdrive.utils.space import ParameterSpace


class BaseRunnable(Configurable, Nameable, Randomizable):
    """
    Abstract class, all sub class must implement all methods to participate in the program running loop
    """
    PARAMETER_SPACE = ParameterSpace({})

    def __init__(self, name=None, random_seed=None, config=None):
        Nameable.__init__(self, name)
        Randomizable.__init__(self, random_seed)
        Configurable.__init__(self, {k: None for k in self.PARAMETER_SPACE.parameters})
        # Parameter check
        assert isinstance(
            self.PARAMETER_SPACE, ParameterSpace
        ), "Using PGSpace to define parameter spaces of " + self.class_name
        self.sample_parameters()
        if config is not None:
            self.update_config(config, allow_add_new_key=True)

    def get_state(self) -> Dict:
        """
        Store current state, for example if this runnable instance is an object in the 3D-world state can be heading,
        position, etc. This function can be used to to store the movement and change history trajectory.
        :return: state dict
        """
        raise NotImplementedError

    def set_state(self, state: Dict):
        """
        Set state for this runnable instance, restore the instance to a certain state, For example, if this runnable
        instance is a policy, it can restore the policy to a certain state to make sure it do the same decision as
        before
        :param state: dict
        """
        raise NotImplementedError

    def before_step(self, *args, **kwargs):
        """
        Do Information fusion and then analyze and wait for decision
        """
        pass

    def set_action(self, *args, **kwargs):
        """
        Set action for this object, and the action will last for the minimal simulation interval
        """
        raise NotImplementedError

    def step(self, *args, **kwargs):
        """
        Call this function to implement the decision set by set_action() for a period of time. This function is usually
        useless, since the result of action, mostly force, is calculated bu game engine via force calculation respect to
        time. However some runnable instances who don't belong to the physics world and their actions are not force need
        to implement this function to get the action accumulated result respect to time.
        """
        pass

    def after_step(self, *args, **kwargs):
        """
        After advancing all objects for a time period, their state should be updated for statistic or other purpose
        """

    def reset(self, *args, **kwargs):
        """
        Although some elements do not need to call reset, please still state this function in it :)
        """
        raise NotImplementedError

    def sample_parameters(self):
        """
        Fix a value of the random parameters in PARAMETER_SPACE
        """
        random_seed = self.np_random.randint(low=0, high=int(1e6))
        self.PARAMETER_SPACE.seed(random_seed)
        ret = self.PARAMETER_SPACE.sample()
        self.update_config(ret)

    def destroy(self):
        Configurable.destroy(self)
