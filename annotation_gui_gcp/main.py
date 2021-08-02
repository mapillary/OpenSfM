import argparse
import json
import shutil
import tkinter as tk
import typing as t
from collections import OrderedDict, defaultdict
from pathlib import Path

import numpy as np
from opensfm import dataset, io

from annotation_gui_gcp.lib import GUI
from annotation_gui_gcp.lib.gcp_manager import GroundControlPointManager
from annotation_gui_gcp.lib.image_manager import ImageManager


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
        "-o",
        "--ortho",
        type=str,
        action="append",
        help="Specify one or more directories containing geotiffs",
        default=[],
    )
    parser.add_argument(
        "--cad",
        type=str,
        help="Specify a directory containing CAD files in FBX format",
        default=None,
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


def update_reconstruction_and_assignments_to_new_format(root: Path):
    """
    The format for the rigs was updated at some point.
    The tool now expects reconstructions and rig assignments to be in the new format
    This function updates these two files if needed, creating a backup as well:
      - reconstruction.json
      - rig_assignments.json
    """

    # Reconstruction
    p_reconstruction = root / "reconstruction.json"
    if p_reconstruction.exists():
        recs_raw = json.load(open(p_reconstruction))
        need_dump = False
        for ix_rec, rec in enumerate(recs_raw):
            if "rig_models" in rec:
                print(f"Updating rig format of {p_reconstruction}")
                need_dump = True
                recs_raw[ix_rec]["rig_cameras"] = {}
                for rig_model in rec["rig_models"].values():
                    recs_raw[ix_rec]["rig_cameras"].update(rig_model["rig_cameras"])
                del rec["rig_models"]

        if need_dump:
            p_backup = root / "reconstruction_old_rigs_format.json"
            shutil.copy(p_reconstruction, p_backup)
            with open(p_reconstruction, "w") as f:
                json.dump(recs_raw, f, indent=4, sort_keys=True)

    # Assignments file
    p_assignments = root / "rig_assignments.json"
    if p_assignments.exists():
        assignments_raw = json.load(open(p_assignments))
        if isinstance(assignments_raw, dict):
            print(f"Updating rig format of {p_assignments}")
            new_assignments = []
            for rig_assignment in assignments_raw.values():
                new_assignments.extend(rig_assignment)
            p_backup = root / "rig_assignments_old_rigs_format.json"
            shutil.copy(p_assignments, p_backup)
            with open(p_assignments, "w") as f:
                json.dump(new_assignments, f, indent=4, sort_keys=True)


def load_rig_assignments(root: Path) -> t.Dict[str, t.List[str]]:
    """
    Returns a dict mapping every shot to all the other corresponding shots in the rig
    """
    update_reconstruction_and_assignments_to_new_format(root)
    p_json = root / "rig_assignments.json"
    if not p_json.exists():
        return {}

    output = {}
    assignments = json.load(open(p_json))
    for shot_group in assignments:
        group_shot_ids = [s[0] for s in shot_group]
        for shot_id, _ in shot_group:
            output[shot_id] = group_shot_ids

    return output


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

    map_key_to_skey = {}
    if groups_from_sequence_database:
        for skey, keys in groups_from_sequence_database.items():
            for k in keys:
                map_key_to_skey[k] = skey

    groups = defaultdict(list)
    for ix_rec, rec_shots in enumerate(all_recs_shots):
        for key in rec_shots:
            if key in map_key_to_skey:
                group_key = f"REC#{ix_rec}_{map_key_to_skey[key]}"
            else:
                group_key = f"REC#{ix_rec}"
            groups[group_key].append(key)

    return groups


def group_images(args):
    """
    Groups the images to be shown in different windows/views

    If group_by_reconstruction is set, each reconstruction will have its own view
    If there is a sequence_database file, each sequence will have its own view

    If group_by_reconstruction is set and there is a sequence_database file, the views
    will be split by sequence and also by reconstruction. For example, if there is a camera
    rig (4 sequences) that is reconstructed into two disjoint reconstructions, you end up
    with 8 views.
    """
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
        return groups_from_sequence_database


def find_suitable_cad_paths(path_cad_files, path_dataset, n_paths=3):
    if path_cad_files is None:
        return []

    def latlon_from_meta(path_cad):
        path_meta = path_cad.with_suffix(".json")
        meta = json.load(open(path_meta))
        return meta["center"]["latitude"], meta["center"]["longitude"]

    # Returns the top N cad models sorted by distance to the dataset
    path_cad_files = Path(path_cad_files)
    cad_files = list(
        set(
            list(path_cad_files.glob("**/*.FBX"))
            + list(path_cad_files.glob("**/*.fbx"))
        )
    )

    lla = dataset.DataSet(path_dataset).load_reference_lla()
    ref_latlon = np.array([lla["latitude"], lla["longitude"]])
    cad_latlons = np.array([latlon_from_meta(path) for path in cad_files])
    distances = np.linalg.norm(cad_latlons - ref_latlon, axis=1)

    ixs_sort = np.argsort(distances)[:n_paths]
    return [cad_files[i] for i in ixs_sort]


if __name__ == "__main__":
    args = parse_args()
    path = args.dataset
    rig_groups = load_rig_assignments(Path(args.dataset))
    groups = group_images(args)
    image_manager = ImageManager(
        groups,
        path,
        preload_images=not args.no_preload,
        max_image_size=args.max_image_size,
    )
    gcp_manager = GroundControlPointManager(path)
    root = tk.Tk()
    root.resizable(True, True)

    ui = GUI.Gui(
        root,
        gcp_manager,
        image_manager,
        rig_groups,
        args.ortho,
        find_suitable_cad_paths(args.cad, path),
    )
    root.grid_columnconfigure(0, weight=1)
    root.grid_rowconfigure(0, weight=1)
    root.title("Tools")
    root.mainloop()
