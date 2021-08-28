from argoverse.data_loading.frame_label_accumulator import PerFrameLabelAccumulator
import pandas as pd
import numpy as np
from pgdrive.constants import ARGOVERSE_AGENT_ID
import os
import pickle

def parse_tracking_data(dataset_dir, log_id):
    pfa = PerFrameLabelAccumulator(dataset_dir, dataset_dir, "")
    pfa.accumulate_per_log_data(log_id = log_id)
    log_egopose_dict = pfa.log_egopose_dict[log_id]
    log_timestamp_dict = pfa.log_timestamp_dict[log_id]

    timesteps = sorted(log_egopose_dict.keys())

    self_id = ARGOVERSE_AGENT_ID
    locate_info = {}
    locate_info[self_id] = {
        "init_pos": None,
        "diag_len": None,
        "traj": {}
    }

    for timestep_index, timestep in enumerate(timesteps):
        xcenter = log_egopose_dict[timestep]['translation'][0]
        ycenter = log_egopose_dict[timestep]['translation'][1]
        if locate_info[self_id]["init_pos"] is None:
            locate_info[self_id]["init_pos"] = np.array([xcenter, -ycenter])
            locate_info[self_id]["diag_len"] = 100
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
                    "diag_len": np.linalg.norm(bbox_city_fr[0, :2] - bbox_city_fr[-1, :2]),
                    "traj": {}
                }
                continue
            locate_info[uuid]["traj"][str(timestep_index)] = center_point

    moving_obj_threshold = 3
    diag_len_threshold = 3
    for key in list(locate_info.keys()):
        traj = locate_info[key]["traj"]
        # print(locate_info[key]["diag_len"])
        if len(traj.keys()) == 0:
            locate_info.pop(key)
            continue
        # Remove static objects
        # min_key = min(traj.keys())
        # max_key = max(traj.keys())
        # crit1 = np.linalg.norm(traj[min_key]-traj[max_key]) < moving_obj_threshold
        # # Remove objects that reappears
        # # crit2 = int((info['end_t'] - info['start_t']) / 1e8) != len(info['traj']) 
        crit3 = locate_info[key]["diag_len"] < 3
        # # if crit1 or crit2:
        if crit3:
            locate_info.pop(key)

    # for key in list(locate_info.keys()):
        # info = locate_info[key]
        # # compute policy inc
        # info["traj"] = np.diff(np.vstack((info["init_pos"], info["traj"])), axis=0)
    with open(os.path.join(dataset_dir, log_id, "city_info.json"), 'r') as f:
        city_info = eval(f.readline())
    city = city_info["city_name"]
    return locate_info, city

def parse_forcasting_data(data_path):
    data = np.array(pd.read_csv(data_path))
    locate_info = {}
    timestep = 0
    for entry in data:
        _, v_id, _, x, y, city = entry
        if v_id not in locate_info.keys():
            locate_info[v_id] = {
                "init_pos": np.array([x, -y]),
                "traj": {}
            }
        else:
            locate_info[v_id]["traj"][str(timestep)] = np.array([x, -y])
        if v_id == ARGOVERSE_AGENT_ID:
            timestep += 1

    # moving_obj_threshold = 1
    for key in list(locate_info.keys()):
        traj = locate_info[key]["traj"]
        if len(traj.keys()) == 0:
            locate_info.pop(key)
            # continue
    #     # Remove static objects
    #     min_key = min(traj.keys())
    #     max_key = max(traj.keys())
    #     crit1 = np.linalg.norm(traj[min_key]-traj[max_key]) < moving_obj_threshold
    #     # Remove objects that reappears
    #     # crit2 = int((info['end_t'] - info['start_t']) / 1e8) != len(info['traj']) 
    #     # if crit1 or crit2:
    #     if crit1:
    #         locate_info.pop(key)
    
    return locate_info, city
        

if __name__ == '__main__':
    # file_path = "/home/xuezhenghai/argoverse-api/argoverse-tracking/train1/"
    # output_path = "/home/xuezhenghai/argoverse-api/argoverse-tracking/train_parsed"
    # if not os.path.isdir(output_path):
    #     os.mkdir(output_path)
    # for log in os.listdir(file_path):
    #     print("Parsing log {}".format(log))
    #     locate_info = parse_tracking_data(file_path, log)
    #     with open(os.path.join(output_path, "{}.pkl".format(log)), 'wb') as f:
    #         pickle.dump(locate_info, f)

    parse_forcasting_data("../argoverse-api/forecasting_sample/data/2645.csv")