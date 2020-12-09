"""
This script is used to remove the optimizer state in the checkpoint. So that we can compress 2/3 of the checkpoint size.
Remember to rename checkpoint-XXX.tune_metadata to checkpoint-NEWNAME.tune_metadata
"""

import pickle

if __name__ == '__main__':
    with open("checkpoint-630", "rb") as f:
        data = f.read()

    unpickled = pickle.loads(data)
    worker = pickle.loads(unpickled.pop("worker"))
    worker["state"]["default_policy"].pop("_optimizer_variables")
    pickled_worker = pickle.dumps(worker)

    to_pickled = {"worker": pickled_worker, **unpickled}
    with open("checkpoint-compressed", "wb") as f:
        pickle.dump(to_pickled, f)
