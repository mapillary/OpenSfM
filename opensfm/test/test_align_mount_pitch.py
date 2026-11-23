from typing import Any

import numpy as np
from numpy.typing import NDArray
from opensfm import align, config, pygeometry, pymap, pysfm, reconstruction, types


MOUNT_PITCH_DEG = 10.0
ARBITRARY_SFM_FRAME = (0.3, -0.5, 0.2)


def _pitched_rotation(pitch: float) -> NDArray:
    """World-to-camera rotation of a camera looking along +X, nose-up by pitch."""
    c, s = np.cos(pitch), np.sin(pitch)
    forward = np.array([c, 0.0, s])
    down = np.array([s, 0.0, -c])
    return np.array([np.cross(down, forward), down, forward])


def _straight_drive(
    world_rotation: tuple[float, float, float] = ARBITRARY_SFM_FRAME,
) -> types.Reconstruction:
    """A nose-up camera driving straight along +X, in the given SfM frame."""
    world = pygeometry.Pose(np.array(world_rotation)).get_rotation_matrix()
    rotation = _pitched_rotation(np.radians(MOUNT_PITCH_DEG)) @ world.T

    rec = types.Reconstruction()
    camera = pygeometry.Camera.create_perspective(0.5, 0.0, 0.0)
    camera.id = "camera"
    rec.add_camera(camera)
    for i in range(20):
        pose = pygeometry.Pose()
        pose.set_rotation_matrix(rotation)
        pose.set_origin(0.5 * world @ np.array([i, 0.0, 0.0]))
        shot = rec.create_shot(f"shot{i}", camera.id, pose)
        shot.metadata.capture_time.value = float(i)
        shot.metadata.sequence_key.value = "sequence"
        shot.metadata.orientation.value = 1
        shot.metadata.gps_position.value = np.array([float(i), 0.0, 0.0])
        shot.metadata.gps_accuracy.value = 1.0
    return rec


def _horizontal_prior_config(compensate: bool) -> dict[str, Any]:
    cfg = config.default_config()
    cfg["align_method"] = "orientation_prior"
    cfg["align_orientation_prior"] = "horizontal"
    cfg["align_compensate_mount_pitch"] = compensate
    return cfg


def _pitch_and_roll_deg(shot: pymap.Shot) -> tuple[float, float]:
    rotation = shot.pose.get_rotation_matrix()
    forward = rotation[2]
    up = rotation[:, 2]
    pitch = np.degrees(np.arcsin(forward[2]))
    roll = np.degrees(np.arctan2(up[0], -up[1]))
    return pitch, roll


def test_estimate_mount_pitches() -> None:
    rec = _straight_drive()
    pitches = pysfm.BAHelpers.estimate_mount_pitches(
        rec.map, _horizontal_prior_config(True)
    )
    assert len(pitches) == 20
    assert np.isclose(np.degrees(pitches["shot3"]), MOUNT_PITCH_DEG)


def test_alignment_keeps_compensated_mount_pitch() -> None:
    rec = _straight_drive()
    align.align_reconstruction(rec, [], _horizontal_prior_config(True))
    for shot in rec.shots.values():
        pitch, roll = _pitch_and_roll_deg(shot)
        assert np.isclose(pitch, MOUNT_PITCH_DEG, atol=1e-6)
        assert np.isclose(roll, 0.0, atol=1e-6)


def _bundle_pitch_deg(compensate: bool) -> float:
    rec = _straight_drive(world_rotation=(0.0, 0.0, 0.0))
    reconstruction.bundle(
        rec,
        dict(rec.cameras.items()),
        dict(rec.rig_cameras.items()),
        [],
        _horizontal_prior_config(compensate),
    )
    pitch, _ = _pitch_and_roll_deg(rec.shots["shot5"])
    return pitch


def test_bundle_up_prior_levels_uncompensated_camera() -> None:
    assert np.isclose(_bundle_pitch_deg(compensate=False), 0.0, atol=0.1)


def test_bundle_up_prior_keeps_compensated_mount_pitch() -> None:
    assert np.isclose(_bundle_pitch_deg(compensate=True), MOUNT_PITCH_DEG, atol=1e-3)
