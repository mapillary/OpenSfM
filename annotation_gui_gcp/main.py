import argparse
import tkinter as tk
from collections import OrderedDict, defaultdict
from pathlib import Path

import GUI
from gcp_manager import GroundControlPointManager
from image_manager import ImageManager
from opensfm import dataset, io


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", help="dataset")
    parser.add_argument(
        "-n", "--no-preload", help="skip preloading", action="store_true"
    )
    parser.add_argument(
        "--max-image-size",
        help="maximum cached image size",
        default=1000,
        type=int,
    )
    parser.add_argument(
        "--group-by-reconstruction",
        action="store_true",
        help="If set, the UI will show one window per reconstruction, "
        "otherwise, it will use sequences as specified by 'sequence-file'",
    )
    parser.add_argument(
        "--strict-missing",
        action="store_true",
    )
    parser.add_argument(
        "--min-images-in-reconstruction",
        type=int,
        default=50,
        help="Reconstructions with fewer images than this value will not be "
        "displayed in the UI",
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
        "Can be used multiple times to define several groups. "
        "Groups defined here will be split into different views even if they "
        "belong to the same reconstruction. ",
        default=[],
    )
    parser.add_argument(
        "-o",
        "--ortho",
        type=str,
        action="append",
        help="Specify one or more directories containing geotiffs",
        default=[],
    )
    return parser.parse_args()


def file_sanity_check(root, seq_dict, fname):
    # Images available under ./images for a sanity check
    available_images = {p.name for p in (root / "images").iterdir()}
    keys_in_seq_dict = {im_key for seq_keys in seq_dict.values() for im_key in seq_keys}

    images_not_in_seq_file = available_images - keys_in_seq_dict
    if len(images_not_in_seq_file) > 0:
        print(f"{len(images_not_in_seq_file)} images not in {fname}")

    n_missing = len(keys_in_seq_dict - available_images)
    if n_missing > 0:
        print(f"There are {n_missing} images from {fname} missing in {(root/'images')}")

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
    if not p_json.exists():
        return None
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

    empty_seqs = [skey for skey in seq_dict if not seq_dict[skey]]
    for skey in empty_seqs:
        del seq_dict[skey]

    return seq_dict


def load_shots_from_reconstructions(path, min_ims):
    data = dataset.DataSet(path)
    reconstructions = data.load_reconstruction()

    # Replace sequence keys for those in sequence_database.json

    n_recs = len(reconstructions)
    if len(reconstructions) > 2:
        reconstructions = [
            r
            for ix_r, r in enumerate(reconstructions)
            if len(r.shots) >= min_ims or ix_r < 2
        ]
    if len(reconstructions) < n_recs:
        print(
            "Kept {}/{} reconstructions (min images: {})".format(
                len(reconstructions),
                n_recs,
                min_ims,
            )
        )

    output = []
    for rec in reconstructions:
        shots = sorted(
            rec.shots.values(), key=lambda x: (x.metadata.capture_time.value, x.id)
        )
        output.append([shot.id for shot in shots])
    return output


def group_by_reconstruction(args, groups_from_sequence_database):
    all_recs_shots = load_shots_from_reconstructions(
        path, min_ims=args.min_images_in_reconstruction
    )
    grouped_skeys = [skey for g in args.sequence_group for skey in g]

    map_key_to_skey = {}
    if groups_from_sequence_database:
        for skey, keys in groups_from_sequence_database.items():
            for k in keys:
                map_key_to_skey[k] = skey

    groups = defaultdict(list)
    sequence_groups = []
    for ix_rec, rec_shots in enumerate(all_recs_shots):
        sequence_groups.append(
            [f"REC#{ix_rec}_{skey}" for group in args.sequence_group for skey in group]
        )
        for key in rec_shots:
            skey = map_key_to_skey.get(key)
            if skey in grouped_skeys:
                # If this sequence is in any of the sequence groups, we want to
                # re-split each reconstruction into different views if they belong
                # to a sequence group. (Typical case: a rig of cameras)
                group_key = f"REC#{ix_rec}_{skey}"
            else:
                group_key = f"REC#{ix_rec}"
            groups[group_key].append(key)

    return groups, sequence_groups


def group_images(args):
    groups_from_sequence_database = load_sequence_database_from_file(
        args.dataset,
        args.sequence_file,
        skip_missing=not args.strict_missing,
    )
    if args.group_by_reconstruction:
        return group_by_reconstruction(args, groups_from_sequence_database)
    else:
        # We only group by sequence key
        if groups_from_sequence_database is None:
            print(
                f"No sequence database file at {args.sequence_file}"
                " and --group-by-reconstruction is disabled. Quitting"
            )
            exit()
        return groups_from_sequence_database, args.sequence_group


if __name__ == "__main__":
    args = parse_args()
    path = args.dataset
    groups, sequence_groups = group_images(args)
    image_manager = ImageManager(
        groups,
        path,
        preload_images=not args.no_preload,
        max_image_size=args.max_image_size,
    )
    gcp_manager = GroundControlPointManager(path)
    root = tk.Tk()
    root.resizable(True, True)
    ui = GUI.Gui(root, gcp_manager, image_manager, sequence_groups, args.ortho, "cad_demo_path.cadfile")
    root.grid_columnconfigure(0, weight=1)
    root.grid_rowconfigure(0, weight=1)
    root.title("Tools")
    root.mainloop()
