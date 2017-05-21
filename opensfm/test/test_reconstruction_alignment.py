
import numpy as np
from opensfm import types
from opensfm import csfm


def get_shot_origin(shot):
    """Compute the origin of a shot."""
    pose = types.Pose([shot.rx, shot.ry, shot.rz], [shot.tx, shot.ty, shot.tz])
    return pose.get_origin()


def get_reconstruction_origin(r):
    """Compute the origin of a reconstruction."""
    s = r.scale
    pose = types.Pose([r.rx, r.ry, r.rz], [r.tx / s, r.ty / s, r.tz / s])
    return pose.get_origin()


def test_single_shot():
    """Single shot test."""
    ra = csfm.ReconstructionAlignment()
    ra.add_shot('1', 0.5, 0, 0, 0, 0, 0, False)
    ra.add_absolute_position_constraint('1', 1, 0, 0, 1)

    ra.run()
    s1 = ra.get_shot('1')
    assert np.allclose(get_shot_origin(s1), [1, 0, 0], atol=1e-6)


def test_singleton_reconstruction():
    """Single shot in a single reconstruction."""
    ra = csfm.ReconstructionAlignment()
    ra.add_shot('1', 0, 0, 0, 0, 0, 0, False)
    ra.add_reconstruction('a', 0, 0, 0, 0, 0, 0, 4, False)
    ra.add_relative_motion_constraint(
        csfm.RARelativeMotionConstraint('a', '1', 0, 0, 0, -1, 0, 0))
    ra.add_absolute_position_constraint('1', 1, 0, 0, 1)

    ra.run()
    s1 = ra.get_shot('1')

    assert np.allclose(get_shot_origin(s1), [1, 0, 0], atol=1e-6)


def test_pair():
    """Simple single reconstruction two shots test."""
    ra = csfm.ReconstructionAlignment()
    ra.add_shot('1', 0, 0, 0, 0, 0, 0, False)
    ra.add_shot('2', 0, 0, 0, 0, 0, 0, False)
    ra.add_reconstruction('a', 0, 0, 0, 0, 0, 0, 4, False)
    ra.add_relative_motion_constraint(
        csfm.RARelativeMotionConstraint('a', '1', 0, 0, 0, 0, 0, 0))
    ra.add_relative_motion_constraint(
        csfm.RARelativeMotionConstraint('a', '2', 0, 0, 0, -1, 0, 0))
    ra.add_absolute_position_constraint('1', 1, 0, 0, 1)
    ra.add_absolute_position_constraint('2', 3, 0, 0, 1)

    ra.run()
    s1 = ra.get_shot('1')
    s2 = ra.get_shot('2')
    rec_a = ra.get_reconstruction('a')

    assert np.allclose(get_shot_origin(s1), [1, 0, 0], atol=1e-6)
    assert np.allclose(get_shot_origin(s2), [3, 0, 0], atol=1e-6)
    assert np.allclose(get_reconstruction_origin(rec_a), [1, 0, 0], atol=1e-6)
    assert np.allclose(rec_a.scale, 0.5)


def test_two_shots_one_fixed():
    """Two shot, one reconstruction. One shot is fixed"""
    ra = csfm.ReconstructionAlignment()
    ra.add_shot('1', 0, 0, 0, -1, 0, 0, True)
    ra.add_shot('2', 0, 0, 0, 0, 0, 0, False)
    ra.add_reconstruction('a', 0, 0, 0, 0, 0, 0, 1, False)
    ra.add_relative_motion_constraint(
        csfm.RARelativeMotionConstraint('a', '1', 0, 0, 0, 0, 0, 0))
    ra.add_relative_motion_constraint(
        csfm.RARelativeMotionConstraint('a', '2', 0, 0, 0, -1, 0, 0))
    # Next line should be ignored because shot 1 is fixed
    ra.add_absolute_position_constraint('1', 100, 0, 0, 1)
    ra.add_absolute_position_constraint('2', 3, 0, 0, 1)

    ra.run()
    s1 = ra.get_shot('1')
    s2 = ra.get_shot('2')
    rec_a = ra.get_reconstruction('a')

    assert np.allclose(get_shot_origin(s1), [1, 0, 0], atol=1e-6)
    assert np.allclose(get_shot_origin(s2), [3, 0, 0], atol=1e-6)
    assert np.allclose(get_reconstruction_origin(rec_a), [1, 0, 0], atol=1e-6)
    assert np.allclose(rec_a.scale, 0.5)


def test_two_reconstructions():
    """Two reconstructions"""
    ra = csfm.ReconstructionAlignment()
    ra.add_shot('1', 0, 0, 0, 0, 0, 0, False)
    ra.add_shot('2', 0, 0, 0, 0, 0, 0, False)
    ra.add_shot('3', 0, 0, 0, 0, 0, 0, False)
    ra.add_shot('4', 0, 0, 0, 0, 0, 0, False)
    ra.add_reconstruction('a', 0, 0, 0, 0, 0, 0, 1, False)
    ra.add_relative_motion_constraint(
        csfm.RARelativeMotionConstraint('a', '1', 0, 0, 0, 0, 0, 0))
    ra.add_relative_motion_constraint(
        csfm.RARelativeMotionConstraint('a', '2', 0, 0, 0, -1, 0, 0))
    ra.add_relative_motion_constraint(
        csfm.RARelativeMotionConstraint('a', '3', 0, 0, 0, -2, 0, 0))
    ra.add_reconstruction('b', 0, 0, 0, 0, 0, 0, 1, False)
    ra.add_relative_motion_constraint(
        csfm.RARelativeMotionConstraint('b', '2', 0, 0, 0, 0, 0, 0))
    ra.add_relative_motion_constraint(
        csfm.RARelativeMotionConstraint('b', '3', 0, 0, 0, -1, 0, 0))
    ra.add_relative_motion_constraint(
        csfm.RARelativeMotionConstraint('b', '4', 0, 0, 0, -2, 0, 0))

    ra.add_absolute_position_constraint('1', 1, 0, 0, 1)
    ra.add_absolute_position_constraint('2', 2, 0, 0, 1)

    ra.run()
    s1 = ra.get_shot('1')
    s2 = ra.get_shot('2')
    s3 = ra.get_shot('3')
    s4 = ra.get_shot('4')
    rec_a = ra.get_reconstruction('a')
    rec_b = ra.get_reconstruction('b')

    assert np.allclose(get_shot_origin(s1), [1, 0, 0], atol=1e-6)
    assert np.allclose(get_shot_origin(s2), [2, 0, 0], atol=1e-6)
    assert np.allclose(get_shot_origin(s3), [3, 0, 0], atol=1e-6)
    assert np.allclose(get_shot_origin(s4), [4, 0, 0], atol=1e-6)
    assert np.allclose(get_reconstruction_origin(rec_a), [1, 0, 0], atol=1e-6)
    assert np.allclose(get_reconstruction_origin(rec_b), [2, 0, 0], atol=1e-6)
    assert np.allclose(rec_a.scale, 1)
    assert np.allclose(rec_b.scale, 1)


def test_common_points():
    """Two reconstructions, two common points"""
    ra = csfm.ReconstructionAlignment()
    ra.add_reconstruction('a', 0, 0, 0, 0, 0, 0, 1, True)
    ra.add_reconstruction('b', 0, 0, 0, 0, 0, 0, 1, False)
    ra.add_common_point_constraint('a', 0, 0, 0, 'b', -1, 0, 0, 1.0)
    ra.add_common_point_constraint('a', 1, 0, 0, 'b', 0, 0, 0, 1.0)

    ra.run()
    rec_b = ra.get_reconstruction('b')

    o_b = get_reconstruction_origin(rec_b)
    assert np.allclose(o_b, [1, 0, 0], atol=1e-6)
