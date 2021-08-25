from pgdrive.policy.base_policy import BasePolicy
from copy import deepcopy

class ReplayPolicy(BasePolicy):
    def __init__(self, control_object, locate_info):
        # Since control object may change
        # TODO: pass in the vehicle id or the whole trajectory
        super(ReplayPolicy, self).__init__(control_object=control_object)
        self.traj_info = locate_info["traj"]
        self.init_pos = locate_info["init_pos"]
        self.cur_pos = deepcopy(self.init_pos)
        self.start_t = locate_info["start_t"]
        self.end_t = locate_info["end_t"]
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

        this_step = self.timestep - self.start_t
        if this_step == 0:
            self.control_object.set_position([self.init_pos[0], self.init_pos[1]])
        elif 0 < this_step < len(self.traj_info):
            self.cur_pos += self.traj_info[this_step - 1]
            self.control_object.set_position(self.cur_pos)

        # TODO: set precise heading
        lane = self.control_object.lane
        heading = lane.heading_at(lane.local_coordinates(self.cur_pos)[0])
        self.control_object.set_heading(heading)

        return [0, 0]