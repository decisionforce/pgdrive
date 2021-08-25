from argoverse.data_loading.frame_label_accumulator import PerFrameLabelAccumulator
import numpy as np
from pgdrive.constants import ARGOVERSE_AGENT_ID

def parse_tracking_data(dataset_dir, log_id):
    pfa = PerFrameLabelAccumulator(dataset_dir, dataset_dir, "")
    pfa.accumulate_per_log_data(log_id = log_id)
    log_egopose_dict = pfa.log_egopose_dict[log_id]
    log_timestamp_dict = pfa.log_timestamp_dict[log_id]

    timesteps = sorted(log_egopose_dict.keys())

    self_id = ARGOVERSE_AGENT_ID
    locate_info = {}
    locate_info[self_id] = {
        "start_t": 0,
        "end_t": len(timesteps)-1,
        "init_pos": None,
        "traj": []
    }

    for timestep_index, timestep in enumerate(timesteps):
        xcenter = log_egopose_dict[timestep]['translation'][0]
        ycenter = log_egopose_dict[timestep]['translation'][1]
        if locate_info[self_id]["init_pos"] is None:
            locate_info[self_id]["init_pos"] = np.array([xcenter, -ycenter])
        else:
            locate_info[self_id]["traj"].append(np.array([xcenter, -ycenter]))
        
        for i, frame_rec in enumerate(log_timestamp_dict[timestep]):
            bbox_city_fr = frame_rec.bbox_city_fr
            uuid = frame_rec.track_uuid
            # TODO: compute heading
            # * heading可以直接设置成与车道线平行
            center_point = np.mean([bbox_city_fr[0, :2], bbox_city_fr[1, :2]], axis = 0)
            center_point *= np.array([1, -1])
            if uuid not in list(locate_info.keys()):
                locate_info[uuid] = {
                    # TODO: change to relative timestep. The agent appears at timestep 0.
                    "start_t": timestep_index,
                    "end_t": timestep_index,
                    "init_pos": center_point,
                    "traj": []
                }
                continue
            locate_info[uuid]["traj"].append(center_point)
            locate_info[uuid]["end_t"] = timestep_index

    moving_obj_threshold = 10
    for key in list(locate_info.keys()):
        info = locate_info[key]
        info["traj"] = np.array(info["traj"])
        traj = info["traj"]
        if len(traj) == 0:
            locate_info.pop(key)
            continue
        # Remove static objects
        crit1 = np.linalg.norm(traj[0]-traj[-1]) < moving_obj_threshold
        # Remove objects that reappears
        crit2 = int((info['end_t'] - info['start_t']) / 1e8) != len(info['traj']) 
        # if crit1 or crit2:
        if crit1:
            locate_info.pop(key)

    for key in list(locate_info.keys()):
        info = locate_info[key]
        # compute policy inc
        info["traj"] = np.diff(np.vstack((info["init_pos"], info["traj"])), axis=0)
    return locate_info

if __name__ == '__main__':
    locate_info = parse_tracking_data("/home/xzh/Research/code/argoverse-api/argoverse-tracking/sample", "c6911883-1843-3727-8eaa-41dc8cda8993")
