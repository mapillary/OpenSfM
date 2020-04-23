from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from __future__ import unicode_literals

from opensfm import dataset
import pickle


def get_most_common_seqs(path):
    data = dataset.DataSet(path)
    reconstructions = data.load_reconstruction()
    seq_dict = {}
    seq_dict_most_common = [[] for i in range(2)]
    for recons in reconstructions:
        for shot in recons.shots.values():
            if not seq_dict.get(shot.metadata.skey, 0):
                seq_dict[shot.metadata.skey] = []
            seq_dict[shot.metadata.skey].append(shot)
    seq_dict_count = sorted(seq_dict, key=lambda k: len(seq_dict[k]),
                            reverse=True)  # two sequences with the most number of images
    for idx, key in enumerate(seq_dict_count[:2]):
        seq_dict_most_common[idx] = sorted(seq_dict[key], key=lambda x: x.metadata.capture_time)
        seq_dict_most_common[idx] = [shot.id for shot in seq_dict_most_common[idx]]

    return seq_dict_most_common


def load_from_pair_list(path):
    with open(path, "rb") as file:
        return pickle.load(file)


def get_all_images(path):
    data = dataset.DataSet(path)
    reconstructions = data.load_reconstruction()
    shots = sorted([shot for reconstruction in reconstructions for shot in reconstruction.shots.values()],
                   key=lambda x: (x.metadata.skey, x.metadata.capture_time))
    seqs = [[shot.id for shot in shots] for i in range(2)]
    return seqs
