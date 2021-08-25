from argoverse.data_loading.frame_label_accumulator import PerFrameLabelAccumulator
import numpy as np
from pgdrive.constants import ARGOVERSE_AGENT_ID
import os
import pickle

def parse_tracking_data(dataset_dir, log_id):
    pfa = PerFrameLabelAccumulator(dataset_dir, dataset_dir, "")
    pfa.accumulate_per_log_data(log_id = log_id)
    log_egopose_dict = pfa.log_egopose_dict[log_id]
    log_timestamp_dict = pfa.log_timestamp_dict[log_id]
    per_city_traj_dict = pfa.per_city_traj_dict
    for city, traj in zip(per_city_traj_dict.keys(), per_city_traj_dict.values()):
        if len(traj) != 0:
            targ_city = city

    timesteps = sorted(log_egopose_dict.keys())

    self_id = ARGOVERSE_AGENT_ID
    locate_info = {}
    locate_info[self_id] = {
        "init_pos": None,
        "traj": {}
    }

    for timestep_index, timestep in enumerate(timesteps):
        xcenter = log_egopose_dict[timestep]['translation'][0]
        ycenter = log_egopose_dict[timestep]['translation'][1]
        if locate_info[self_id]["init_pos"] is None:
            locate_info[self_id]["init_pos"] = np.array([xcenter, -ycenter])
        else:
            locate_info[self_id]["traj"][str(timestep_index)] = np.array([xcenter, -ycenter])
        
        for i, frame_rec in enumerate(log_timestamp_dict[timestep]):
            bbox_city_fr = frame_rec.bbox_city_fr
            uuid = frame_rec.track_uuid
            # TODO: compute heading
            # * heading可以直接设置成与车道线平行
            center_point = np.mean([bbox_city_fr[0, :2], bbox_city_fr[-1, :2]], axis = 0)
            center_point *= np.array([1, -1])
            if uuid not in list(locate_info.keys()):
                locate_info[uuid] = {
                    "init_pos": center_point,
                    "traj": {}
                }
                continue
            locate_info[uuid]["traj"][str(timestep_index)] = center_point

    moving_obj_threshold = 10
    for key in list(locate_info.keys()):
        traj = locate_info[key]["traj"]
        if len(traj.keys()) == 0:
            locate_info.pop(key)
            continue
        # Remove static objects
        min_key = min(traj.keys())
        max_key = max(traj.keys())
        crit1 = np.linalg.norm(traj[min_key]-traj[max_key]) < moving_obj_threshold
        # Remove objects that reappears
        # crit2 = int((info['end_t'] - info['start_t']) / 1e8) != len(info['traj']) 
        # if crit1 or crit2:
        if crit1:
            locate_info.pop(key)

    # for key in list(locate_info.keys()):
        # info = locate_info[key]
        # # compute policy inc
        # info["traj"] = np.diff(np.vstack((info["init_pos"], info["traj"])), axis=0)
    return locate_info, targ_city

if __name__ == '__main__':
    file_path = "/home/xuezhenghai/argoverse-api/argoverse-tracking/train1/"
    output_path = "/home/xuezhenghai/argoverse-api/argoverse-tracking/train_parsed"
    if not os.path.isdir(output_path):
        os.mkdir(output_path)
    for log in os.listdir(file_path):
        print("Parsing log {}".format(log))
        locate_info = parse_tracking_data(file_path, log)
        with open(os.path.join(output_path, "{}.pkl".format(log)), 'wb') as f:
            pickle.dump(locate_info, f)
