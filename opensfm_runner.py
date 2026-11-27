"""
opensfm_runner — standalone OpenSfM pipeline binary
=====================================================

SYNOPSIS
    opensfm_runner <command> <dataset> [options]
    opensfm_runner -h | --help

COMMANDS
    run               Run the full pipeline end-to-end (default)
    extract_metadata  Extract EXIF metadata and camera models from images
    detect_features   Detect and describe keypoint features in each image
    match_features    Match features across overlapping image pairs
    create_tracks     Link matched features into 3-D tracks
    reconstruct       Run incremental Structure-from-Motion reconstruction
    export_ply        Export the reconstruction to a PLY point-cloud file
    test              Run the built-in test suite against a dataset

PIPELINE ORDER
    extract_metadata → detect_features → match_features →
    create_tracks → reconstruct → export_ply

OUTPUT FILES  (all written inside <dataset>/)
    exif/<image>.exif            — per-image EXIF metadata (extract_metadata)
    camera_models.json           — camera model catalogue (extract_metadata)
    features/<image>.features.npz— keypoints + descriptors (detect_features)
    reference_lla.json           — GPS reference point  (match_features)
    matches/<image>_matches.pkl.gz — pairwise matches   (match_features)
    tracks.csv                   — feature tracks       (create_tracks)
    reconstruction.json          — 3-D reconstruction   (reconstruct)
    reconstruction.ply           — point cloud          (export_ply)

EXAMPLES
    # Full pipeline
    opensfm_runner run data/lund

    # Single step
    opensfm_runner extract_metadata data/lund
    opensfm_runner detect_features  data/lund
    opensfm_runner match_features   data/lund
    opensfm_runner create_tracks    data/lund
    opensfm_runner reconstruct      data/lund
    opensfm_runner export_ply       data/lund

    # Test suite (cleans existing outputs first by default)
    opensfm_runner test data/lund
    opensfm_runner test data/lund --no-clean

    # Full pipeline with JSON progress output (for programmatic use)
    opensfm_runner run data/lund --json

NOTES
    - Progress is printed as human-readable text by default.
    - Use --json to emit newline-delimited JSON for programmatic consumers
      (e.g. a Tauri desktop app reading stdout).
    - "WARNING: free: command not found" is harmless on macOS; OpenSfM
      tries to read available RAM via a Linux-only utility.
    - The 'single-line alignment' warning means the dataset images were
      captured along a roughly linear path; a horizontal prior is used.
"""

import sys
import json
import os
import argparse
import multiprocessing
import glob

# Required for PyInstaller + multiprocessing on macOS
multiprocessing.freeze_support()

# Fix path for PyInstaller bundle
if getattr(sys, 'frozen', False):
    bundle_dir = sys._MEIPASS
    sys.path.insert(0, bundle_dir)

from opensfm.dataset import DataSet
from opensfm.actions import (
    extract_metadata,
    detect_features,
    match_features,
    create_tracks,
    reconstruct,
    export_ply,
)
from opensfm.reconstruction import ReconstructionAlgorithm


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------

def _emit(msg, as_json, **extra):
    if as_json:
        print(json.dumps({"message": msg, **extra}), flush=True)
    else:
        print(msg, flush=True)

def _progress(step, as_json):
    if as_json:
        print(json.dumps({"status": "progress", "step": step}), flush=True)
    else:
        print(f"[{step}]", flush=True)

def _done(dataset_path, as_json):
    if as_json:
        print(json.dumps({"status": "done", "path": dataset_path}), flush=True)
    else:
        print(f"Done: {dataset_path}", flush=True)

def _error(message, as_json):
    if as_json:
        print(json.dumps({"status": "error", "message": message}), flush=True)
    else:
        print(f"ERROR: {message}", file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------
# Individual step runners
# ---------------------------------------------------------------------------

def step_extract_metadata(dataset, as_json=False):
    _progress("Extracting metadata", as_json)
    extract_metadata.run_dataset(dataset)

def step_detect_features(dataset, as_json=False):
    _progress("Detecting features", as_json)
    detect_features.run_dataset(dataset)

def step_match_features(dataset, as_json=False):
    _progress("Matching features", as_json)
    match_features.run_dataset(dataset)

def step_create_tracks(dataset, as_json=False):
    _progress("Creating tracks", as_json)
    create_tracks.run_dataset(dataset)

def step_reconstruct(dataset, as_json=False):
    _progress("Reconstructing", as_json)
    reconstruct.run_dataset(dataset, ReconstructionAlgorithm.INCREMENTAL)

def step_export_ply(dataset, as_json=False):
    _progress("Exporting PLY", as_json)
    export_ply.run_dataset(
        dataset,
        no_cameras=False,
        no_points=False,
        depthmaps=False,
        point_num_views=False,
    )


STEPS = [
    ("extract_metadata", step_extract_metadata),
    ("detect_features",  step_detect_features),
    ("match_features",   step_match_features),
    ("create_tracks",    step_create_tracks),
    ("reconstruct",      step_reconstruct),
    ("export_ply",       step_export_ply),
]


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def cmd_run(dataset_path, as_json=False):
    dataset = DataSet(dataset_path)
    for _, fn in STEPS:
        fn(dataset, as_json)
    _done(dataset_path, as_json)


def cmd_single_step(step_name, dataset_path, as_json=False):
    dataset = DataSet(dataset_path)
    fn = dict(STEPS)[step_name]
    fn(dataset, as_json)
    _done(dataset_path, as_json)


# ---------------------------------------------------------------------------
# Test suite
# ---------------------------------------------------------------------------

class TestResult:
    def __init__(self):
        self.passed = []
        self.failed = []

    def check(self, description, condition, detail=""):
        if condition:
            self.passed.append(description)
            print(f"  PASS  {description}", flush=True)
        else:
            self.failed.append(description)
            msg = f"  FAIL  {description}"
            if detail:
                msg += f"\n        → {detail}"
            print(msg, flush=True)

    def summary(self):
        total = len(self.passed) + len(self.failed)
        print(flush=True)
        print(f"Results: {len(self.passed)}/{total} passed", flush=True)
        if self.failed:
            print("Failed:", flush=True)
            for f in self.failed:
                print(f"  - {f}", flush=True)
        return len(self.failed) == 0


def cmd_test(dataset_path, clean=True):
    if not os.path.isdir(dataset_path):
        print(f"ERROR: dataset path does not exist: {dataset_path}", file=sys.stderr)
        sys.exit(1)

    images_dir = os.path.join(dataset_path, "images")
    if not os.path.isdir(images_dir):
        print(f"ERROR: no images/ directory in {dataset_path}", file=sys.stderr)
        sys.exit(1)

    images = sorted(
        f for f in os.listdir(images_dir)
        if f.lower().endswith((".jpg", ".jpeg", ".png", ".tif", ".tiff"))
    )
    if not images:
        print(f"ERROR: no images found in {images_dir}", file=sys.stderr)
        sys.exit(1)

    print(f"Dataset : {dataset_path}", flush=True)
    print(f"Images  : {len(images)}", flush=True)

    # Optionally clean computed outputs (keep images, config, camera_models)
    if clean:
        print("Cleaning previous outputs ...", flush=True)
        for d in ("exif", "features", "matches", "reports"):
            import shutil
            target = os.path.join(dataset_path, d)
            if os.path.isdir(target):
                shutil.rmtree(target)
        for f in ("tracks.csv", "reconstruction.json", "reconstruction.ply",
                  "reference_lla.json"):
            p = os.path.join(dataset_path, f)
            if os.path.isfile(p):
                os.remove(p)

    r = TestResult()
    dataset = DataSet(dataset_path)

    # --- extract_metadata ---
    print("\n[1/6] extract_metadata", flush=True)
    try:
        extract_metadata.run_dataset(dataset)
        exif_files = glob.glob(os.path.join(dataset_path, "exif", "*.exif"))
        r.check("exif/ dir created", os.path.isdir(os.path.join(dataset_path, "exif")))
        r.check(
            f"exif file count matches image count ({len(images)})",
            len(exif_files) == len(images),
            f"found {len(exif_files)} exif files",
        )
        r.check(
            "camera_models.json created",
            os.path.isfile(os.path.join(dataset_path, "camera_models.json")),
        )
    except Exception as e:
        r.check("extract_metadata ran without exception", False, str(e))

    # --- detect_features ---
    print("\n[2/6] detect_features", flush=True)
    try:
        detect_features.run_dataset(dataset)
        feat_files = glob.glob(os.path.join(dataset_path, "features", "*.features.npz"))
        r.check("features/ dir created", os.path.isdir(os.path.join(dataset_path, "features")))
        r.check(
            f"feature file count matches image count ({len(images)})",
            len(feat_files) == len(images),
            f"found {len(feat_files)} feature files",
        )
        # Spot-check: first feature file is non-empty
        if feat_files:
            r.check(
                "first feature file is non-empty",
                os.path.getsize(feat_files[0]) > 0,
                feat_files[0],
            )
    except Exception as e:
        r.check("detect_features ran without exception", False, str(e))

    # --- match_features ---
    print("\n[3/6] match_features", flush=True)
    try:
        match_features.run_dataset(dataset)
        match_files = glob.glob(os.path.join(dataset_path, "matches", "*_matches.pkl.gz"))
        r.check("matches/ dir created", os.path.isdir(os.path.join(dataset_path, "matches")))
        r.check(
            "at least one match file produced",
            len(match_files) > 0,
            f"found {len(match_files)} match files",
        )
        r.check(
            "reference_lla.json created",
            os.path.isfile(os.path.join(dataset_path, "reference_lla.json")),
        )
    except Exception as e:
        r.check("match_features ran without exception", False, str(e))

    # --- create_tracks ---
    print("\n[4/6] create_tracks", flush=True)
    try:
        create_tracks.run_dataset(dataset)
        tracks_path = os.path.join(dataset_path, "tracks.csv")
        r.check("tracks.csv created", os.path.isfile(tracks_path))
        if os.path.isfile(tracks_path):
            r.check(
                "tracks.csv is non-empty",
                os.path.getsize(tracks_path) > 0,
            )
    except Exception as e:
        r.check("create_tracks ran without exception", False, str(e))

    # --- reconstruct ---
    print("\n[5/6] reconstruct", flush=True)
    try:
        reconstruct.run_dataset(dataset, ReconstructionAlgorithm.INCREMENTAL)
        recon_path = os.path.join(dataset_path, "reconstruction.json")
        r.check("reconstruction.json created", os.path.isfile(recon_path))
        if os.path.isfile(recon_path):
            with open(recon_path) as f:
                data = json.load(f)
            r.check(
                "reconstruction.json contains at least one reconstruction",
                isinstance(data, list) and len(data) > 0,
                f"found {len(data) if isinstance(data, list) else '?'} reconstruction(s)",
            )
            if isinstance(data, list) and data:
                shots = data[0].get("shots", {})
                r.check(
                    f"reconstruction contains shots ({len(shots)} cameras placed)",
                    len(shots) > 0,
                )
    except Exception as e:
        r.check("reconstruct ran without exception", False, str(e))

    # --- export_ply ---
    print("\n[6/6] export_ply", flush=True)
    try:
        export_ply.run_dataset(
            dataset,
            no_cameras=False,
            no_points=False,
            depthmaps=False,
            point_num_views=False,
        )
        ply_path = os.path.join(dataset_path, "reconstruction.ply")
        r.check("reconstruction.ply created", os.path.isfile(ply_path))
        if os.path.isfile(ply_path):
            size = os.path.getsize(ply_path)
            r.check(
                f"reconstruction.ply is non-empty ({size} bytes)",
                size > 0,
            )
    except Exception as e:
        r.check("export_ply ran without exception", False, str(e))

    ok = r.summary()
    sys.exit(0 if ok else 1)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

HELP_TEXT = __doc__

def build_parser():
    parser = argparse.ArgumentParser(
        prog="opensfm_runner",
        description="Standalone OpenSfM pipeline binary",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=HELP_TEXT,
        add_help=True,
    )

    sub = parser.add_subparsers(dest="command", metavar="<command>")

    # run (full pipeline)
    p_run = sub.add_parser("run", help="Run the full pipeline end-to-end")
    p_run.add_argument("dataset", help="Path to the dataset directory")
    p_run.add_argument("--json", action="store_true", help="Emit newline-delimited JSON progress")

    # individual steps
    for step_name, _ in STEPS:
        p = sub.add_parser(step_name, help=f"Run only the {step_name} step")
        p.add_argument("dataset", help="Path to the dataset directory")
        p.add_argument("--json", action="store_true", help="Emit newline-delimited JSON progress")

    # test
    p_test = sub.add_parser("test", help="Run test suite and validate outputs")
    p_test.add_argument("dataset", help="Path to the dataset directory")
    p_test.add_argument(
        "--no-clean",
        action="store_true",
        help="Do not remove existing outputs before testing",
    )

    return parser


def main():
    parser = build_parser()

    # If called with no arguments, print help
    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(0)

    # Legacy: if first arg looks like a path (not a known command), run full pipeline
    known_commands = {"run", "test"} | {name for name, _ in STEPS}
    if sys.argv[1] not in known_commands and not sys.argv[1].startswith("-"):
        # Treat as: opensfm_runner <dataset>  (original behaviour, JSON output)
        dataset_path = sys.argv[1]
        cmd_run(dataset_path, as_json=True)
        return

    args = parser.parse_args()

    if args.command == "run":
        cmd_run(args.dataset, as_json=args.json)
    elif args.command == "test":
        cmd_test(args.dataset, clean=not args.no_clean)
    elif args.command in dict(STEPS):
        cmd_single_step(args.command, args.dataset, as_json=args.json)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
