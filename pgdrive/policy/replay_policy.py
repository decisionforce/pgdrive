from pgdrive.policy.base_policy import BasePolicy
from copy import deepcopy
has_rendered = False

class ReplayPolicy(BasePolicy):
    def __init__(self, control_object, locate_info):
        super(ReplayPolicy, self).__init__(control_object=control_object)
        self.traj_info = locate_info["traj"]
        self.start_index = min(self.traj_info.keys())
        self.init_pos = locate_info["init_pos"]
        self.timestep = 0
        self.damp = 0
        # how many times the replay data is slowed down
        self.damp_interval = 1

    def act(self, *args, **kwargs):
        self.damp += self.damp_interval
        if self.damp == self.damp_interval:
            self.timestep += 1
            self.damp = 0
        else:
            return [0,0]

        if str(self.timestep) == self.start_index:
            self.control_object.set_position(self.init_pos)
        elif str(self.timestep) in self.traj_info.keys():
            self.control_object.set_position(self.traj_info[str(self.timestep)])
        # else:
        #     self.control_object.set_position((0, 0))
            

        # TODO: set precise heading
        lane = self.control_object.lane
        heading = lane.heading_at(lane.local_coordinates(self.control_object.position)[0])
        self.control_object.set_heading(heading)

        return [0, 0]
