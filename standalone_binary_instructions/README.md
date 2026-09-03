# opensfm_runner

Standalone OpenSfM pipeline binary for macOS (arm64 / Apple Silicon).  
No Python installation or dependencies required.

---

## Requirements

- macOS on Apple Silicon (arm64)
- The binary is self-contained — no Python, conda, or pip needed

---

## Quick Start

```bash
# Make executable (first time only)
chmod +x ./opensfm_runner

# Full help manual
./opensfm_runner --help

# Run the full pipeline on a dataset
./opensfm_runner run /path/to/dataset
```

A **dataset directory** must contain:
- `images/` — JPEG/PNG/TIFF source photos
- `config.yaml` — OpenSfM configuration (optional; defaults are used if absent)

---

## Commands

### Full pipeline

Runs all six steps in order and writes all output files to the dataset directory.

```bash
./opensfm_runner run /path/to/dataset
```

Add `--json` for newline-delimited JSON progress (useful for programmatic consumers such as a Tauri app reading stdout):

```bash
./opensfm_runner run /path/to/dataset --json
```

### Individual steps

Run a single step independently. Steps must be executed in pipeline order.

```bash
./opensfm_runner extract_metadata /path/to/dataset
./opensfm_runner detect_features  /path/to/dataset
./opensfm_runner match_features   /path/to/dataset
./opensfm_runner create_tracks    /path/to/dataset
./opensfm_runner reconstruct      /path/to/dataset
./opensfm_runner export_ply       /path/to/dataset
```

Each step also accepts `--json` for JSON output.

### Pipeline order

```
extract_metadata → detect_features → match_features →
create_tracks → reconstruct → export_ply
```

---

## Output Files

All outputs are written inside the dataset directory:

| File | Created by |
|---|---|
| `exif/<image>.exif` | extract_metadata |
| `camera_models.json` | extract_metadata |
| `features/<image>.features.npz` | detect_features |
| `reference_lla.json` | match_features |
| `matches/<image>_matches.pkl.gz` | match_features |
| `tracks.csv` | create_tracks |
| `reconstruction.json` | reconstruct |
| `reconstruction.ply` | export_ply |

---

## Running the Test Suite

The `test` command runs all six pipeline steps from scratch against a dataset and validates that every expected output file is produced correctly. It runs 16 checks in total.

```bash
# Clean run — removes previous outputs first, then runs all steps
./opensfm_runner test /path/to/dataset

# Validate existing outputs without re-running (no clean)
./opensfm_runner test /path/to/dataset --no-clean
```

### Example output

```
Dataset : data/lund
Images  : 29
Cleaning previous outputs ...

[1/6] extract_metadata
  PASS  exif/ dir created
  PASS  exif file count matches image count (29)
  PASS  camera_models.json created

[2/6] detect_features
  PASS  features/ dir created
  PASS  feature file count matches image count (29)
  PASS  first feature file is non-empty

[3/6] match_features
  PASS  matches/ dir created
  PASS  at least one match file produced
  PASS  reference_lla.json created

[4/6] create_tracks
  PASS  tracks.csv created
  PASS  tracks.csv is non-empty

[5/6] reconstruct
  PASS  reconstruction.json created
  PASS  reconstruction.json contains at least one reconstruction
  PASS  reconstruction contains shots (29 cameras placed)

[6/6] export_ply
  PASS  reconstruction.ply created
  PASS  reconstruction.ply is non-empty (3891148 bytes)

Results: 16/16 passed
```

Exit code is `0` on full pass, `1` if any check fails.

---

## Known Warnings (harmless)

| Warning | Cause | Impact |
|---|---|---|
| `/bin/sh: free: command not found` | OpenSfM checks available RAM via a Linux-only command | None — ignored on macOS |
| `Shots aligned on a single-line. Using horizontal prior` | Dataset images were captured in a nearly straight line | None — a fallback alignment prior is used automatically |

---

## Troubleshooting

**`zlib.error: unknown compression method` on startup**

This happens when macOS reuses a stale extraction cache from a previous version of the binary. Clear it with:

```bash
find "$TMPDIR" -maxdepth 1 -name '_MEI*' -exec rm -rf {} +
```

Then run the binary again.

**Fewer shots placed than images in the dataset**

This is normal for datasets with a linear capture path (e.g. a straight walkway). The incremental reconstructor may exclude frames that don't meet geometric constraints. The reconstruction is still valid.

---

## Rebuilding the Binary

If you modify `opensfm_runner.py`, rebuild with:

```bash
CENV=/opt/homebrew/Cellar/micromamba/2.5.0_4/envs/opensfm
PYTHONPATH=/path/to/OpenSfM \
$CENV/bin/pyinstaller \
  --onefile --name opensfm_runner \
  --paths /path/to/OpenSfM \
  --collect-all opensfm --collect-all yaml --collect-all pyproj \
  --collect-all fpdf --collect-all scipy \
  --hidden-import xmltodict \
  --hidden-import opensfm.actions.extract_metadata \
  --hidden-import opensfm.actions.detect_features \
  --hidden-import opensfm.actions.match_features \
  --hidden-import opensfm.actions.create_tracks \
  --hidden-import opensfm.actions.reconstruct \
  --hidden-import opensfm.actions.export_ply \
  opensfm_runner.py
```

Then clear the extraction cache before testing the new build:

```bash
find "$TMPDIR" -maxdepth 1 -name '_MEI*' -exec rm -rf {} +
```
