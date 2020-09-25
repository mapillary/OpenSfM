import argparse
import tkinter as tk
from collections import OrderedDict, defaultdict
from pathlib import Path

from Database import Database
from GUI import Gui
from opensfm import io, dataset


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", help="dataset")
    parser.add_argument(
        "-n", "--no-preload", help="skip preloading", action="store_true"
    )
    parser.add_argument(
        "--group-by-reconstruction",
        action="store_true",
        help="If set, the UI will show one window per reconstruction, "
        "otherwise, it will use sequences as specified by 'sequence-file'",
    )
    parser.add_argument(
        "--sequence-file",
        help="dict-of-image-keys JSON file describing each sequence. "
             "Format: {'sequence_key': ['im_key_1', 'im_key_2', ...], ...}",
        default="sequence_database.json",
    )
    parser.add_argument(
        "-sg",
        "--sequence-group",
        type=str,
        nargs="+",
        action="append",
        help="Specify one or more groups of linked sequences. "
        "Linked sequences are synchronized such that all views "
        "from the group will always show the same frame index. "
        "Useful for camera rigs. usage: -g sequence_key_1 sequence_key_2. "
        "Can be used multiple times to define several groups",
        default=[],
    )
    return parser.parse_args()


def file_sanity_check(root, seq_dict, fname):
    # Images available under ./images for a sanity check
    available_images = set([p.name for p in (root / "images").iterdir()])
    keys_in_seq_dict = set(
        [im_key for seq_keys in seq_dict.values() for im_key in seq_keys]
    )

    images_not_in_seq_file = available_images - keys_in_seq_dict
    if len(images_not_in_seq_file) > 0:
        print(f"{len(images_not_in_seq_file)} images not in {fname}")

    missing_image_files = keys_in_seq_dict - available_images
    if len(missing_image_files) > 0:
        print(
            f"There are {len(missing_image_files)} images from {fname} missing in {(root/'images')}"
        )

    return available_images


def load_sequence_database_from_file(
    root, fname="sequence_database.json", skip_missing=False
):
    """
    Simply loads a sequence file and returns it.
    This doesn't require an existing SfM reconstruction
    """
    root = Path(root)
    p_json = root / fname
    assert p_json.exists()
    seq_dict = OrderedDict(io.json_load(open(p_json, "r")))

    available_images = file_sanity_check(root, seq_dict, fname)

    for skey in seq_dict:
        available_image_keys = []
        for k in seq_dict[skey]:
            if k in available_images:
                available_image_keys.append(k)
            elif not skip_missing:
                raise FileNotFoundError(f"{k} not found")
        seq_dict[skey] = available_image_keys
    return seq_dict


def group_images_by_reconstruction(path):
    data = dataset.DataSet(path)
    reconstructions = data.load_reconstruction()
    seq_dict = defaultdict(list)
    for recons in reconstructions:
        for shot in recons.shots.values():
            skey = shot.metadata.sequence_key.value
            seq_dict[skey].append(shot)
    for skey in seq_dict:
        sorted_ims = sorted(seq_dict[skey], key=lambda x: x.metadata.capture_time.value)
        seq_dict[skey] = [shot.id for shot in sorted_ims]
    return seq_dict


if __name__ == "__main__":
    args = parse_args()
    path = args.dataset
    if args.group_by_reconstruction:
        seqs = group_images_by_reconstruction(path)
    else:
        seqs = load_sequence_database_from_file(path, args.sequence_file, skip_missing=True)
    database = Database(seqs, path, preload_images=not args.no_preload)
    root = tk.Tk()
    root.resizable(True, True)
    my_gui = Gui(root, database, len(seqs), args.sequence_group)
    root.grid_columnconfigure(0, weight=1)
    root.grid_rowconfigure(0, weight=1)
    root.title("Tools")
    root.mainloop()
