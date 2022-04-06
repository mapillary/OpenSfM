import argparse
import json
import typing as t
from collections import OrderedDict, defaultdict
from os import PathLike
from pathlib import Path
from typing import Union

import numpy as np
from annotation_gui_gcp.lib import GUI
from annotation_gui_gcp.lib.gcp_manager import GroundControlPointManager
from annotation_gui_gcp.lib.image_manager import ImageManager
from flask import Flask
from opensfm import dataset, io


def get_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", help="dataset")
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
        "--cad",
        type=str,
        help="Specify a directory containing CAD files in FBX format",
        default=None,
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5000,
    )
    return parser


def file_sanity_check(root, seq_dict, fname) -> t.Set[str]:
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


def load_rig_assignments(root: Path) -> t.Dict[str, t.List[str]]:
    """
    Returns a dict mapping every shot to all the other corresponding shots in the rig
    """
    p_json = root / "rig_assignments.json"
    if not p_json.exists():
        return {}

    output = {}
    with open(p_json) as f:
        assignments: t.Dict[str, t.List[t.Tuple[str, str]]] = json.load(f)
    for shot_group in assignments.values():
        group_shot_ids = [s[0] for s in shot_group]
        for shot_id, _ in shot_group:
            output[shot_id] = group_shot_ids

    return output


def load_sequence_database_from_file(
    root: Path,
    fname: Union["PathLike[str]", str] = "sequence_database.json",
    skip_missing: bool = False,
):
    """
    Simply loads a sequence file and returns it.
    This doesn't require an existing SfM reconstruction
    """
    root = Path(root)
    p_json = root / fname
    if not p_json.exists():
        return None
    with open(p_json) as f:
        seq_dict = OrderedDict(io.json_load(f))

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


def load_shots_from_reconstructions(path, min_ims) -> t.List[t.List[str]]:
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


def group_by_reconstruction(
    args, groups_from_sequence_database
) -> t.Dict[str, t.List[str]]:
    all_recs_shots = load_shots_from_reconstructions(
        args.dataset, min_ims=args.min_images_in_reconstruction
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


def group_images(args) -> t.Dict[str, t.List[str]]:
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


def find_suitable_cad_paths(path_cad_files: Path, path_dataset, n_paths: int = 6):
    if path_cad_files is None:
        return []

    def latlon_from_meta(path_cad) -> t.Tuple[float, float]:
        path_meta = path_cad.with_suffix(".json")
        with open(path_meta) as f:
            meta = json.load(f)
        return meta["center"]["latitude"], meta["center"]["longitude"]

    # Returns the top N cad models sorted by distance to the dataset
    path_cad_files = Path(path_cad_files)
    cad_files = list(
        set(
            list(path_cad_files.glob("**/*.FBX"))
            + list(path_cad_files.glob("**/*.fbx"))
        )
    )

    ref = dataset.DataSet(path_dataset).load_reference()
    ref_latlon = np.array([ref.lat, ref.lon])
    cad_latlons = np.array([latlon_from_meta(path) for path in cad_files])
    distances = np.linalg.norm(cad_latlons - ref_latlon, axis=1)

    ixs_sort = np.argsort(distances)[:n_paths]
    return [cad_files[i] for i in ixs_sort]


def init_ui() -> t.Tuple[Flask, argparse.Namespace]:
    app = Flask(__name__)
    parser = get_parser()
    args = parser.parse_args()
    path = args.dataset
    rig_groups = load_rig_assignments(Path(args.dataset))
    groups = group_images(args)
    image_manager = ImageManager(
        groups,
        path,
    )
    gcp_manager = GroundControlPointManager(path)
    GUI.Gui(
        app,
        gcp_manager,
        image_manager,
        rig_groups,
        find_suitable_cad_paths(args.cad, path, 1),
    )
    return app, args


if __name__ == "__main__":
    app, args = init_ui()
    app.run(host="::", port=args.port)
