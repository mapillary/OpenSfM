import copy
import numpy as np

from opensfm import csfm
from opensfm import geometry
from opensfm import config
from opensfm import types
from opensfm import reconstruction


def test_unicode_strings_in_bundle():
    """Test that byte and unicode strings can be used as camera ids."""
    ba = csfm.BundleAdjuster()

    unicode_id = u"A\xb2"
    byte_id = b"A_2"

    ba.add_equirectangular_camera(unicode_id)
    ba.add_equirectangular_camera(byte_id)


def test_sigleton():
    """Single camera test"""
    sa = csfm.BundleAdjuster()
    sa.add_shot('1', 'cam1', [0.5, 0, 0], [0, 0, 0], False)
    sa.add_absolute_position('1', [1, 0, 0], 1, '1')
    sa.add_absolute_up_vector('1', [0, -1, 0], 1)
    sa.add_absolute_pan('1', np.radians(180), 1)

    sa.run()
    s1 = sa.get_shot('1')
    assert np.allclose(s1.t, [1, 0, 0], atol=1e-6)


def test_sigleton_pan_tilt_roll():
    """Single camera test with pan, tilt, roll priors."""
    pan, tilt, roll = 1, 0.3, 0.2
    sa = csfm.BundleAdjuster()
    sa.add_shot('1', 'cam1', [0.5, 0, 0], [0, 0, 0], False)
    sa.add_absolute_position('1', [1, 0, 0], 1, '1')
    sa.add_absolute_pan('1', pan, 1)
    sa.add_absolute_tilt('1', tilt, 1)
    sa.add_absolute_roll('1', roll, 1)

    sa.run()
    s1 = sa.get_shot('1')
    pose = types.Pose(s1.r, s1.t)

    assert np.allclose(pose.get_origin(), [1, 0, 0], atol=1e-6)

    ptr = geometry.ptr_from_rotation(pose.get_rotation_matrix())
    assert np.allclose(ptr, (pan, tilt, roll))


def _projection_errors_std(points):
    all_errors = []
    for p in points.values():
        all_errors += p.reprojection_errors.values()
    return np.std(all_errors)


def test_bundle_projection_fixed_internals(scene_synthetic):
    reference = scene_synthetic[0].get_reconstruction()
    graph = scene_synthetic[5]
    adjusted = copy.deepcopy(reference)

    custom_config = config.default_config()
    custom_config['bundle_use_gps'] = False
    custom_config['optimize_camera_parameters'] = False
    reconstruction.bundle(graph, adjusted, {}, custom_config)

    assert _projection_errors_std(adjusted.points) < 5e-3
    assert reference.cameras['1'].focal == adjusted.cameras['1'].focal
    assert reference.cameras['1'].k1 == adjusted.cameras['1'].k1
    assert reference.cameras['1'].k2 == adjusted.cameras['1'].k2


def test_pair():
    """Simple two camera test"""
    sa = csfm.BundleAdjuster()
    sa.add_shot('1', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_shot('2', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_reconstruction('12', False)
    sa.add_reconstruction_shot('12', 4, '1')
    sa.add_reconstruction_shot('12', 4, '2')
    sa.set_scale_sharing('12', True)
    sa.add_relative_motion(csfm.BARelativeMotion('12', '1', '12', '2', [0, 0, 0], [-1, 0, 0]))
    sa.add_absolute_position('1', [0, 0, 0], 1, '1')
    sa.add_absolute_position('2', [2, 0, 0], 1, '2')

    sa.run()
    s1 = sa.get_shot('1')
    s2 = sa.get_shot('2')
    r12 = sa.get_reconstruction('12')

    assert np.allclose(s1.t, [0, 0, 0], atol=1e-6)
    assert np.allclose(s2.t, [-2, 0, 0], atol=1e-6)
    assert np.allclose(r12.get_scale('1'), 0.5)
    assert np.allclose(r12.get_scale('2'), 0.5)


def test_pair_with_shot_point():
    """Simple two camera test with a point constraint for anchoring"""
    sa = csfm.BundleAdjuster()
    sa.add_shot('1', 'cam1', [0, 0, 0], [1e-3, 1e-3, 1e-3], False)
    sa.add_shot('2', 'cam1', [0, 0, 0], [1e-3, 1e-3, 1e-3], False)
    sa.add_point('p1', [0, 0, 0], False)
    sa.add_reconstruction('12', False)
    sa.add_reconstruction_shot('12', 4, '1')
    sa.add_reconstruction_shot('12', 4, '2')
    sa.set_scale_sharing('12', True)
    sa.add_relative_motion(csfm.BARelativeMotion('12', '1', '12', '2', [0, 0, 0], [-1, 0, 0]))
    sa.add_point_position_shot('p1', '1', '12', [1, 0, 0], 1,  csfm.XYZ)
    sa.add_point_position_shot('p1', '2', '12', [-1, 0, 0], 1,  csfm.XYZ)
    sa.add_point_bearing_shot('p1', '1', '12', [1, 0, 0], 2e-3)
    sa.add_point_position_world('p1', [1, 0, 0], 1,  csfm.XYZ)

    sa.run()
    s1 = sa.get_shot('1')
    s2 = sa.get_shot('2')
    r12 = sa.get_reconstruction('12')
    p1 = sa.get_point('p1')

    assert np.allclose(s1.t, [0.5, 0, 0], atol=1e-2)
    assert np.allclose(s2.t, [-1.5, 0, 0], atol=1e-2)
    assert np.allclose(p1.p, [1, 0, 0], atol=1e-6)
    assert np.allclose(r12.get_scale('1'), 0.5)
    assert np.allclose(r12.get_scale('2'), 0.5)


def test_pair_non_rigid():
    """Simple two camera test"""
    sa = csfm.BundleAdjuster()
    sa.add_shot('1', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_shot('2', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_reconstruction('12', False)
    sa.add_reconstruction_shot('12', 4, '1')
    sa.add_reconstruction_shot('12', 4, '2')
    sa.set_scale_sharing('12', False)
    sa.add_relative_similarity(csfm.BARelativeSimilarity('12', '1', '12', '2', [0, 0, 0], [-1, 0, 0], 1))
    sa.add_absolute_position('1', [0, 0, 0], 1, '1')
    sa.add_absolute_position('2', [2, 0, 0], 1, '2')

    sa.run()
    s1 = sa.get_shot('1')
    s2 = sa.get_shot('2')
    r12 = sa.get_reconstruction('12')

    assert np.allclose(s1.t, [0, 0, 0], atol=1e-6)
    assert np.allclose(s2.t, [-2, 0, 0], atol=1e-6)
    assert np.allclose(r12.get_scale('1'), 0.5)
    assert np.allclose(r12.get_scale('2'), 0.5)


def test_four_cams_single_reconstruction():
    """Four cameras, one reconstruction"""
    sa = csfm.BundleAdjuster()
    sa.add_shot('1', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_shot('2', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_shot('3', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_shot('4', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_reconstruction('1234', False)
    sa.add_reconstruction_shot('1234', 1, '1')
    sa.add_reconstruction_shot('1234', 1, '2')
    sa.add_reconstruction_shot('1234', 1, '3')
    sa.add_reconstruction_shot('1234', 1, '4')
    sa.set_scale_sharing('1234', True)
    sa.add_relative_motion(csfm.BARelativeMotion('1234', '1', '1234', '2', [0, 0, 0], [-1, 0, 0]))
    sa.add_relative_motion(csfm.BARelativeMotion('1234', '1', '1234', '3', [0, 0, 0], [0, -1, 0]))
    sa.add_relative_motion(csfm.BARelativeMotion('1234', '1', '1234', '4', [0, 0, 0], [0, 0, -1]))
    sa.add_absolute_position('1', [0, 0, 0], 1, '1')
    sa.add_absolute_position('2', [2, 0, 0], 1, '2')
    sa.add_absolute_position('3', [0, 2, 0], 1, '3')

    sa.run()
    s1 = sa.get_shot('1')
    s2 = sa.get_shot('2')
    s3 = sa.get_shot('3')
    s4 = sa.get_shot('4')

    r1234 = sa.get_reconstruction('1234')

    assert np.allclose(s1.t, [0, 0, 0], atol=1e-6)
    assert np.allclose(s2.t, [-2, 0, 0], atol=1e-6)
    assert np.allclose(s3.t, [0, -2, 0], atol=1e-6)
    assert np.allclose(s4.t, [0, 0, -2], atol=1e-6)


def test_four_cams_single_reconstruction_non_rigid():
    """Four cameras, one reconstruction"""
    sa = csfm.BundleAdjuster()
    sa.add_shot('1', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_shot('2', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_shot('3', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_shot('4', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_reconstruction('1234', False)
    sa.add_reconstruction_shot('1234', 1, '1')
    sa.add_reconstruction_shot('1234', 1, '2')
    sa.add_reconstruction_shot('1234', 1, '3')
    sa.add_reconstruction_shot('1234', 1, '4')
    sa.set_scale_sharing('1234', False)

    sa.add_relative_similarity(csfm.BARelativeSimilarity('1234', '1', '1234', '2', [0, 0, 0], [-1, 0, 0], 1))
    sa.add_relative_similarity(csfm.BARelativeSimilarity('1234', '2', '1234', '3', [0, 0, 0], [-1, -1, 0], 1))
    sa.add_relative_similarity(csfm.BARelativeSimilarity('1234', '3', '1234', '4', [0, 0, 0], [0, -1, 0], 1))
    sa.add_absolute_position('1', [0, 0, 0], 1, '1')
    sa.add_absolute_position('2', [2, 0, 0], 1, '2')
    sa.add_absolute_position('3', [4, 2, 0], 1, '3')
    sa.add_absolute_position('4', [4, 4, 0], 1, '4')

    sa.run()
    s1 = sa.get_shot('1')
    s2 = sa.get_shot('2')
    s3 = sa.get_shot('3')
    s4 = sa.get_shot('4')

    r1234 = sa.get_reconstruction('1234')

    assert np.allclose(s1.t, [0, 0, 0], atol=1e-6)
    assert np.allclose(s2.t, [-2, 0, 0], atol=1e-6)
    assert np.allclose(s3.t, [-4, -2, 0], atol=1e-6)
    assert np.allclose(s4.t, [-4, -4, 0], atol=1e-6)
    assert np.allclose(r1234.get_scale('1'), 0.5)
    assert np.allclose(r1234.get_scale('2'), 0.5)
    assert np.allclose(r1234.get_scale('3'), 0.5)
    assert np.allclose(r1234.get_scale('4'), 0.5)


def test_four_cams_one_fixed():
    """Four cameras, one reconstruction"""
    sa = csfm.BundleAdjuster()
    sa.add_shot('1', 'cam1', [0, 0, 0], [0, 0, 0], True)
    sa.add_shot('2', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_shot('3', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_shot('4', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_reconstruction('1234', False)
    sa.add_reconstruction_shot('1234', 1, '1')
    sa.add_reconstruction_shot('1234', 1, '2')
    sa.add_reconstruction_shot('1234', 1, '3')
    sa.add_reconstruction_shot('1234', 1, '4')
    sa.set_scale_sharing('1234', True)
    sa.add_relative_motion(csfm.BARelativeMotion('1234', '1', '1234', '2', [0, 0, 0], [-1, 0, 0]))
    sa.add_relative_motion(csfm.BARelativeMotion('1234', '1', '1234', '3', [0, 0, 0], [0, -1, 0]))
    sa.add_relative_motion(csfm.BARelativeMotion('1234', '1', '1234', '4', [0, 0, 0], [0, 0, -1]))
    sa.add_absolute_position('1', [100, 0, 0], 1, '1')
    sa.add_absolute_position('2', [2, 0, 0], 1, '2')
    sa.add_absolute_position('3', [0, 2, 0], 1, '3')

    sa.run()
    s1 = sa.get_shot('1')
    s2 = sa.get_shot('2')
    s3 = sa.get_shot('3')
    s4 = sa.get_shot('4')

    assert np.allclose(s1.t, [0, 0, 0], atol=1e-6)
    assert np.allclose(s2.t, [-2, 0, 0], atol=1e-6)
    assert np.allclose(s3.t, [0, -2, 0], atol=1e-6)
    assert np.allclose(s4.t, [0, 0, -2], atol=1e-6)


def test_linear_motion_prior_position():
    """Three cameras, middle has no gps info. Translation only"""
    sa = csfm.BundleAdjuster()
    sa.add_shot('1', 'cam1', [0, 0, 0], [0, 0, 0], True)
    sa.add_shot('2', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_shot('3', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_reconstruction('123', False)
    sa.add_reconstruction_shot('123', 1, '1')
    sa.add_reconstruction_shot('123', 1, '2')
    sa.add_reconstruction_shot('123', 1, '3')
    sa.set_scale_sharing('123', True)
    sa.add_absolute_position('1', [0, 0, 0], 1, '1')
    sa.add_absolute_position('3', [2, 0, 0], 1, '3')
    sa.add_linear_motion('1', '2', '3', 0.5, 0.1, 0.1)

    sa.run()
    s1 = sa.get_shot('1')
    s2 = sa.get_shot('2')
    s3 = sa.get_shot('3')

    assert np.allclose(s1.t, [0, 0, 0], atol=1e-6)
    assert np.allclose(s2.t, [-1, 0, 0], atol=1e-6)
    assert np.allclose(s3.t, [-2, 0, 0], atol=1e-6)


def test_linear_motion_prior_rotation():
    """Three cameras, middle has no gps or orientation info"""
    sa = csfm.BundleAdjuster()
    sa.add_shot('1', 'cam1', [0, 0, 0], [0, 0, 0], True)
    sa.add_shot('2', 'cam1', [0, 0, 0], [0, 0, 0], False)
    sa.add_shot('3', 'cam1', [0, 1, 0], [0, 0, 0], True)
    sa.add_reconstruction('123', False)
    sa.add_reconstruction_shot('123', 1, '1')
    sa.add_reconstruction_shot('123', 1, '2')
    sa.add_reconstruction_shot('123', 1, '3')
    sa.set_scale_sharing('123', True)
    sa.add_linear_motion('1', '2', '3', 0.3, 0.1, 0.1)

    sa.run()
    s2 = sa.get_shot('2')

    assert np.allclose(s2.r, [0, 0.3, 0], atol=1e-6)
