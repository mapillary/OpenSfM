# pyre-unsafe
import copy

import numpy as np
import pytest
from opensfm import (
    config,
    geometry,
    pybundle,
    pygeometry,
    pymap,
    reconstruction,
    tracking,
    types,
)


def test_unicode_strings_in_bundle() -> None:
    """Test that byte and unicode strings can be used as camera ids."""
    ba = pybundle.BundleAdjuster()

    unicode_id = "A\xb2"
    byte_id = b"A_2"

    camera = pygeometry.Camera.create_perspective(0.4, 0.1, -0.01)

    camera.id = unicode_id
    ba.add_camera(camera.id, camera, camera, True)

    # pyre-fixme[8]: Attribute has type `str`; used as `bytes`.
    camera.id = byte_id
    ba.add_camera(camera.id, camera, camera, True)


@pytest.fixture()
def bundle_adjuster() -> pybundle.BundleAdjuster:
    ba = pybundle.BundleAdjuster()
    camera = pygeometry.Camera.create_perspective(1.0, 0.0, 0.0)
    ba.add_camera("cam1", camera, camera, True)
    ba.add_rig_camera("rig_cam1", pygeometry.Pose(), pygeometry.Pose(), True)
    return ba


def test_sigleton(bundle_adjuster: pybundle.BundleAdjuster) -> None:
    """Single camera test"""
    sa = bundle_adjuster
    sa.add_rig_instance(
        "1",
        pygeometry.Pose(np.array([0.5, 0, 0]), np.array([0, 0, 0])),
        {"1": "cam1"},
        {"1": "rig_cam1"},
        False,
    )
    sa.add_rig_instance_position_prior(
        "1", np.array([1, 0, 0]), np.array([1, 1, 1]), ""
    )
    sa.add_absolute_up_vector("1", np.array([0, -1, 0]), 1)
    sa.add_absolute_pan("1", np.radians(180), 1)

    sa.run()
    s1 = sa.get_rig_instance_pose("1")
    assert np.allclose(s1.translation, [1, 0, 0], atol=1e-6)


def test_singleton_pan_tilt_roll(bundle_adjuster: pybundle.BundleAdjuster) -> None:
    """Single camera test with pan, tilt, roll priors."""
    pan, tilt, roll = 1, 0.3, 0.2
    sa = bundle_adjuster
    sa.add_rig_instance(
        "1",
        pygeometry.Pose(np.array([0.5, 0, 0]), np.array([0, 0, 0])),
        {"1": "cam1"},
        {"1": "rig_cam1"},
        False,
    )
    sa.add_rig_instance_position_prior(
        "1", np.array([1, 0, 0]), np.array([1, 1, 1]), ""
    )
    sa.add_absolute_pan("1", pan, 1)
    sa.add_absolute_tilt("1", tilt, 1)
    sa.add_absolute_roll("1", roll, 1)

    sa.run()
    pose = sa.get_rig_instance_pose("1")

    assert np.allclose(pose.get_origin(), [1, 0, 0], atol=1e-6)

    ptr = geometry.ptr_from_rotation(pose.get_rotation_matrix())
    assert np.allclose(ptr, (pan, tilt, roll))


def _projection_errors_std(points) -> float:
    all_errors = []
    for p in points.values():
        all_errors += p.reprojection_errors.values()
    return np.std(all_errors)


def test_bundle_projection_fixed_internals(scene_synthetic) -> None:
    reference = scene_synthetic.reconstruction
    camera_priors = dict(reference.cameras.items())
    rig_priors = dict(reference.rig_cameras.items())
    graph = tracking.as_graph(scene_synthetic.tracks_manager)
    # Create the connections in the reference
    for point_id in reference.points.keys():
        if point_id in graph:
            for shot_id, g_obs in graph[point_id].items():
                color = g_obs["feature_color"]
                pt = g_obs["feature"]
                obs = pymap.Observation(
                    pt[0],
                    pt[1],
                    g_obs["feature_scale"],
                    color[0],
                    color[1],
                    color[2],
                    g_obs["feature_id"],
                    g_obs["feature_segmentation"],
                    g_obs["feature_instance"],
                )
                reference.map.add_observation(shot_id, point_id, obs)

    orig_camera = copy.deepcopy(reference.cameras["1"])

    custom_config = config.default_config()
    custom_config["bundle_use_gps"] = False
    custom_config["optimize_camera_parameters"] = False
    reconstruction.bundle(reference, camera_priors, rig_priors, [], custom_config)

    assert _projection_errors_std(reference.points) < 5e-3
    assert reference.cameras["1"].focal == orig_camera.focal
    assert reference.cameras["1"].k1 == orig_camera.k1
    assert reference.cameras["1"].k2 == orig_camera.k2


def create_shots(bundle_adjuster: pybundle.BundleAdjuster, num_shots: int) -> None:
    for i in range(num_shots):
        instance_id = str(i + 1)
        bundle_adjuster.add_rig_instance(
            instance_id,
            pygeometry.Pose(np.array([0, 0, 0]), np.array([0, 0, 0])),
            {instance_id: "cam1"},
            {instance_id: "rig_cam1"},
            False,
        )


def test_pair(bundle_adjuster: pybundle.BundleAdjuster) -> None:
    """Simple two camera test"""
    sa = bundle_adjuster
    create_shots(sa, 2)

    sa.add_reconstruction("12", False)
    sa.add_reconstruction_instance("12", 4, "1")
    sa.add_reconstruction_instance("12", 4, "2")
    sa.set_scale_sharing("12", True)
    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "1", "2", np.array([0, 0, 0]), np.array([-1, 0, 0]), 1, 1, False
        )
    )

    std_dev = np.array([1, 1, 1])
    sa.add_rig_instance_position_prior("1", np.array([0, 0, 0]), std_dev, "")
    sa.add_rig_instance_position_prior("2", np.array([2, 0, 0]), std_dev, "")

    sa.run()
    s1 = sa.get_rig_instance_pose("1")
    s2 = sa.get_rig_instance_pose("2")
    r12 = sa.get_reconstruction("12")

    assert np.allclose(s1.translation, [0, 0, 0], atol=1e-6)
    assert np.allclose(s2.translation, [-2, 0, 0], atol=1e-6)
    assert np.allclose(r12.get_scale("1"), 0.5)
    assert np.allclose(r12.get_scale("2"), 0.5)


def test_pair_with_points_priors(bundle_adjuster: pybundle.BundleAdjuster) -> None:
    """Simple two rigs test with a point constraint for anchoring"""
    sa = bundle_adjuster
    for i in range(2):
        instance_id = str(i + 1)
        sa.add_rig_instance(
            instance_id,
            pygeometry.Pose(np.array([1e-3, 1e-3, 1e-3]), np.array([1e-3, 1e-3, 1e-3])),
            {instance_id: "cam1"},
            {instance_id: "rig_cam1"},
            False,
        )

    sa.add_point("p1", np.array([0, 0, 0]), False)
    sa.add_point("p2", np.array([0, 0, 0]), False)

    sa.add_reconstruction("12", False)
    sa.add_reconstruction_instance("12", 4, "1")
    sa.add_reconstruction_instance("12", 4, "2")

    # identity rotation with pan/tilt/roll
    sa.add_absolute_roll("1", np.radians(90), 1)
    sa.add_absolute_pan("1", -np.radians(90), 1)
    sa.add_absolute_tilt("1", -np.radians(90), 1)

    sa.set_scale_sharing("12", True)
    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "1",
            "2",
            np.array([0, 0, 0]),
            np.array([-1, 0, 0]),
            1,
            1,
            False,
        )
    )

    std_dev = np.array([1, 1, 1])
    sa.add_point_projection_observation("1", "p1", np.array([0, 0]), 1)
    sa.add_point_projection_observation("2", "p1", np.array([-0.5, 0]), 1)
    sa.add_point_prior("p1", np.array([-0.5, 2, 2]), std_dev, True)

    sa.add_point_projection_observation("2", "p2", np.array([0, 0]), 1)
    sa.add_point_projection_observation("1", "p2", np.array([0.5, 0]), 1)
    sa.add_point_prior("p2", np.array([1.5, 2, 2]), std_dev, True)

    sa.run()
    s1 = sa.get_rig_instance_pose("1")
    s2 = sa.get_rig_instance_pose("2")
    r12 = sa.get_reconstruction("12")
    p1 = sa.get_point("p1")
    p2 = sa.get_point("p2")

    assert np.allclose(s1.translation, [0.5, -2, 2], atol=1e-2)
    assert np.allclose(s2.translation, [-1.5, -2, 2], atol=1e-2)
    assert np.allclose(p1.p, [-0.5, 2, 2], atol=1e-6)
    assert np.allclose(p2.p, [1.5, 2, 2], atol=1e-6)
    assert np.allclose(r12.get_scale("1"), 0.5)
    assert np.allclose(r12.get_scale("2"), 0.5)


def test_pair_non_rigid(bundle_adjuster: pybundle.BundleAdjuster) -> None:
    """Simple two rigs test"""
    sa = bundle_adjuster
    create_shots(sa, 2)

    sa.add_reconstruction("12", False)
    sa.add_reconstruction_instance("12", 4, "1")
    sa.add_reconstruction_instance("12", 4, "2")
    sa.set_scale_sharing("12", False)
    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "1", "2", np.array([0, 0, 0]), np.array([-1, 0, 0]), 1, 1, False
        )
    )

    std_dev = np.array([1, 1, 1])
    sa.add_rig_instance_position_prior("1", np.array([0, 0, 0]), std_dev, "")
    sa.add_rig_instance_position_prior("2", np.array([2, 0, 0]), std_dev, "")

    sa.run()
    s1 = sa.get_rig_instance_pose("1")
    s2 = sa.get_rig_instance_pose("2")
    r12 = sa.get_reconstruction("12")

    assert np.allclose(s1.translation, [0, 0, 0], atol=1e-6)
    assert np.allclose(s2.translation, [-2, 0, 0], atol=1e-6)
    assert np.allclose(r12.get_scale("1"), 4.0)
    assert np.allclose(r12.get_scale("2"), 0.5)


def test_four_cams_single_reconstruction(
    bundle_adjuster: pybundle.BundleAdjuster,
) -> None:
    """Four rigs, one reconstruction"""
    sa = bundle_adjuster
    create_shots(sa, 4)

    sa.add_reconstruction("1234", False)
    sa.add_reconstruction_instance("1234", 1, "1")
    sa.add_reconstruction_instance("1234", 1, "2")
    sa.add_reconstruction_instance("1234", 1, "3")
    sa.add_reconstruction_instance("1234", 1, "4")
    sa.set_scale_sharing("1234", True)

    relative_scale = 1
    robust = 1
    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "1",
            "2",
            np.array([0, 0, 0]),
            np.array([-1, 0, 0]),
            relative_scale,
            robust,
            False,
        )
    )
    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "1",
            "3",
            np.array([0, 0, 0]),
            np.array([0, -1, 0]),
            relative_scale,
            robust,
            False,
        )
    )
    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "1",
            "4",
            np.array([0, 0, 0]),
            np.array([0, 0, -1]),
            relative_scale,
            robust,
            False,
        )
    )

    std_dev = np.array([1, 1, 1])
    sa.add_rig_instance_position_prior("1", np.array([0, 0, 0]), std_dev, "")
    sa.add_rig_instance_position_prior("2", np.array([2, 0, 0]), std_dev, "")
    sa.add_rig_instance_position_prior("3", np.array([0, 2, 0]), std_dev, "")

    sa.run()
    s1 = sa.get_rig_instance_pose("1")
    s2 = sa.get_rig_instance_pose("2")
    s3 = sa.get_rig_instance_pose("3")
    s4 = sa.get_rig_instance_pose("4")

    assert np.allclose(s1.translation, [0, 0, 0], atol=1e-6)
    assert np.allclose(s2.translation, [-2, 0, 0], atol=1e-6)
    assert np.allclose(s3.translation, [0, -2, 0], atol=1e-6)
    assert np.allclose(s4.translation, [0, 0, -2], atol=1e-6)


def test_four_cams_double_reconstruction(
    bundle_adjuster: pybundle.BundleAdjuster,
) -> None:
    """Four rigs, two reconstruction"""
    sa = bundle_adjuster
    create_shots(sa, 4)

    sa.add_reconstruction("12", False)
    sa.add_reconstruction_instance("12", 1, "1")
    sa.add_reconstruction_instance("12", 1, "2")
    sa.set_scale_sharing("12", False)

    sa.add_reconstruction("34", False)
    sa.add_reconstruction_instance("34", 1, "3")
    sa.add_reconstruction_instance("34", 1, "4")
    sa.set_scale_sharing("34", False)

    relative_scale = 1
    robust = 1
    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "1",
            "2",
            np.array([0, 0, 0]),
            np.array([-0.5, -0.5, -0.5]),
            relative_scale,
            robust,
            True,
        )
    )
    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "3",
            "2",
            np.array([0, 0, 0]),
            np.array([0.5, 0.5, 0.5]),
            relative_scale,
            robust,
            False,
        )
    )

    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "3",
            "4",
            np.array([0, 0, 0]),
            np.array([-2, -2, -2]),
            relative_scale,
            robust,
            True,
        )
    )
    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "2",
            "3",
            np.array([0, 0, 0]),
            np.array([-2, -2, -2]),
            relative_scale,
            robust,
            False,
        )
    )

    std_dev = np.array([1, 1, 1])
    sa.add_rig_instance_position_prior("1", np.array([0, 0, 0]), std_dev, "")
    sa.add_rig_instance_position_prior("4", np.array([3, 3, 3]), std_dev, "")

    sa.run()
    s1 = sa.get_rig_instance_pose("1")
    s2 = sa.get_rig_instance_pose("2")
    s3 = sa.get_rig_instance_pose("3")
    s4 = sa.get_rig_instance_pose("4")
    r12 = sa.get_reconstruction("12")
    r34 = sa.get_reconstruction("34")

    assert np.allclose(s1.get_origin(), [0, 0, 0], atol=1e-6)
    assert np.allclose(s2.get_origin(), [1, 1, 1], atol=1e-6)
    assert np.allclose(s3.get_origin(), [2, 2, 2], atol=1e-6)
    assert np.allclose(s4.get_origin(), [3, 3, 3], atol=1e-6)

    r12 = sa.get_reconstruction("12")
    assert np.allclose(r12.get_scale("1"), 0.5)
    assert np.allclose(r12.get_scale("2"), 0.5)

    r34 = sa.get_reconstruction("34")
    assert np.allclose(r34.get_scale("3"), 2.0)
    assert np.allclose(r34.get_scale("4"), 2.0)


def test_four_cams_one_fixed(bundle_adjuster: pybundle.BundleAdjuster) -> None:
    """Four rigs, one reconstruction"""
    sa = bundle_adjuster
    for i in range(4):
        instance_id = str(i + 1)
        sa.add_rig_instance(
            instance_id,
            pygeometry.Pose(np.array([0, 0, 0]), np.array([0, 0, 0])),
            {instance_id: "cam1"},
            {instance_id: "rig_cam1"},
            i == 0,
        )

    sa.add_reconstruction("1234", False)
    sa.add_reconstruction_instance("1234", 1, "1")
    sa.add_reconstruction_instance("1234", 1, "2")
    sa.add_reconstruction_instance("1234", 1, "3")
    sa.add_reconstruction_instance("1234", 1, "4")
    sa.set_scale_sharing("1234", True)

    relative_scale = 1
    robust = 1
    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "1",
            "2",
            np.array([0, 0, 0]),
            np.array([-1, 0, 0]),
            relative_scale,
            robust,
            False,
        )
    )
    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "1",
            "3",
            np.array([0, 0, 0]),
            np.array([0, -1, 0]),
            relative_scale,
            robust,
            False,
        )
    )
    sa.add_relative_motion(
        pybundle.RelativeMotion(
            "1",
            "4",
            np.array([0, 0, 0]),
            np.array([0, 0, -1]),
            relative_scale,
            robust,
            False,
        )
    )

    std_dev = np.array([1, 1, 1])
    sa.add_rig_instance_position_prior("1", np.array([100, 0, 0]), std_dev, "")
    sa.add_rig_instance_position_prior("2", np.array([2, 0, 0]), std_dev, "")
    sa.add_rig_instance_position_prior("3", np.array([0, 2, 0]), std_dev, "")

    sa.run()
    s1 = sa.get_rig_instance_pose("1")
    s2 = sa.get_rig_instance_pose("2")
    s3 = sa.get_rig_instance_pose("3")
    s4 = sa.get_rig_instance_pose("4")

    assert np.allclose(s1.translation, [0, 0, 0], atol=1e-6)
    assert np.allclose(s2.translation, [-2, 0, 0], atol=1e-6)
    assert np.allclose(s3.translation, [0, -2, 0], atol=1e-6)
    assert np.allclose(s4.translation, [0, 0, -2], atol=1e-6)


def test_linear_motion_prior_position(bundle_adjuster: pybundle.BundleAdjuster) -> None:
    """Three rigs, middle has no gps info. Translation only"""
    sa = bundle_adjuster
    create_shots(sa, 3)

    sa.add_reconstruction("123", False)
    sa.add_reconstruction_instance("123", 1, "1")
    sa.add_reconstruction_instance("123", 1, "2")
    sa.add_reconstruction_instance("123", 1, "3")
    sa.set_scale_sharing("123", True)

    std_dev = np.array([1, 1, 1])
    sa.add_rig_instance_position_prior("1", np.array([0, 0, 0]), std_dev, "")
    sa.add_rig_instance_position_prior("3", np.array([2, 0, 0]), std_dev, "")
    sa.add_linear_motion("1", "2", "3", 0.5, 0.1, 0.1)

    sa.run()
    s1 = sa.get_rig_instance_pose("1")
    s2 = sa.get_rig_instance_pose("2")
    s3 = sa.get_rig_instance_pose("3")

    assert np.allclose(s1.translation, [0, 0, 0], atol=1e-6)
    assert np.allclose(s2.translation, [-1, 0, 0], atol=1e-6)
    assert np.allclose(s3.translation, [-2, 0, 0], atol=1e-6)


def test_linear_motion_prior_rotation(bundle_adjuster: pybundle.BundleAdjuster) -> None:
    """Three rigs, middle has no gps or orientation info"""
    sa = bundle_adjuster
    sa.add_rig_instance(
        "1",
        pygeometry.Pose(np.array([0, 0, 0]), np.array([0, 0, 0])),
        {"1": "cam1"},
        {"1": "rig_cam1"},
        True,
    )
    sa.add_rig_instance(
        "2",
        pygeometry.Pose(np.array([0, 0, 0]), np.array([0, 0, 0])),
        {"2": "cam1"},
        {"2": "rig_cam1"},
        False,
    )
    sa.add_rig_instance(
        "3",
        pygeometry.Pose(np.array([0, 1, 0]), np.array([0, 0, 0])),
        {"3": "cam1"},
        {"3": "rig_cam1"},
        True,
    )
    sa.add_reconstruction("123", False)
    sa.add_reconstruction_instance("123", 1, "1")
    sa.add_reconstruction_instance("123", 1, "2")
    sa.add_reconstruction_instance("123", 1, "3")
    sa.set_scale_sharing("123", True)
    sa.add_linear_motion("1", "2", "3", 0.3, 0.1, 0.1)

    sa.run()
    s2 = sa.get_rig_instance_pose("2")

    assert np.allclose(s2.rotation, [0, 0.3, 0], atol=1e-6)


def test_bundle_void_gps_ignored() -> None:
    """Test that void gps values are ignored."""
    camera = pygeometry.Camera.create_perspective(1.0, 0.0, 0.0)
    camera.id = "camera1"

    r = types.Reconstruction()
    r.add_camera(camera)
    shot = r.create_shot(
        "1", camera.id, pygeometry.Pose(np.random.rand(3), np.random.rand(3))
    )

    camera_priors = {camera.id: camera}
    rig_priors = dict(r.rig_cameras.items())
    gcp = []
    myconfig = config.default_config()

    # Missing position
    shot.metadata.gps_position.value = np.zeros(3)
    shot.metadata.gps_accuracy.value = 1
    shot.metadata.gps_position.reset()
    shot.pose.set_origin(np.ones(3))
    reconstruction.bundle(r, camera_priors, rig_priors, gcp, myconfig)
    assert np.allclose(shot.pose.get_origin(), np.ones(3))

    # Missing accuracy
    shot.metadata.gps_position.value = np.zeros(3)
    shot.metadata.gps_accuracy.value = 1
    shot.metadata.gps_accuracy.reset()
    shot.pose.set_origin(np.ones(3))
    reconstruction.bundle(r, camera_priors, rig_priors, gcp, myconfig)
    assert np.allclose(shot.pose.get_origin(), np.ones(3))

    # Valid gps position and accuracy
    shot.metadata.gps_position.value = np.zeros(3)
    shot.metadata.gps_accuracy.value = 1
    shot.pose.set_origin(np.ones(3))
    reconstruction.bundle(r, camera_priors, rig_priors, gcp, myconfig)
    assert np.allclose(shot.pose.get_origin(), np.zeros(3))


def test_bundle_alignment_prior() -> None:
    """Test that cameras are aligned to have the Y axis pointing down."""
    camera = pygeometry.Camera.create_perspective(1.0, 0.0, 0.0)
    camera.id = "camera1"

    r = types.Reconstruction()
    r.add_camera(camera)
    shot = r.create_shot(
        "1", camera.id, pygeometry.Pose(np.random.rand(3), np.random.rand(3))
    )
    shot.metadata.gps_position.value = np.array([0, 0, 0])
    shot.metadata.gps_accuracy.value = 1

    camera_priors = {camera.id: camera}
    rig_priors = dict(r.rig_cameras.items())
    gcp = []
    myconfig = config.default_config()

    reconstruction.bundle(r, camera_priors, rig_priors, gcp, myconfig)
    shot = r.shots[shot.id]

    assert np.allclose(shot.pose.translation, np.zeros(3))
    # up vector in camera coordinates is (0, -1, 0)
    assert np.allclose(shot.pose.transform([0, 0, 1]), [0, -1, 0], atol=1e-7)


def test_heatmaps_position(bundle_adjuster: pybundle.BundleAdjuster) -> None:
    """Three cameras. Same heatmap different offsets"""
    sa = bundle_adjuster
    for i in range(3):
        instance_id = str(i + 1)
        sa.add_rig_instance(
            instance_id,
            pygeometry.Pose(np.array([0, 0, 0]), np.array([0, 0, 0])),
            {instance_id: "cam1"},
            {instance_id: "rig_cam1"},
            False,
        )

    sa.add_reconstruction("123", True)
    sa.add_reconstruction_instance("123", 1, "1")
    sa.add_reconstruction_instance("123", 1, "2")
    sa.add_reconstruction_instance("123", 1, "3")
    sa.set_scale_sharing("123", True)

    def bell_heatmap(size, r, mu_x, mu_y):
        sigma_x = r * 0.5
        sigma_y = r * 0.5
        x = np.linspace(-r, r, size)
        y = np.linspace(r, -r, size)

        x, y = np.meshgrid(x, y)
        z = (
            1
            / (2 * np.pi * sigma_x * sigma_y)
            * np.exp(
                -(
                    (x - mu_x) ** 2 / (2 * sigma_x ** 2)
                    + (y - mu_y) ** 2 / (2 * sigma_y ** 2)
                )
            )
        )
        z /= max(z.reshape(-1))
        z = 1 - z
        return z

    hmap_x, hmap_y = 1, -1
    hmap_size, hmap_r = 101, 10
    res = 2 * hmap_r / (hmap_size - 1)
    hmap = bell_heatmap(size=hmap_size, r=hmap_r, mu_x=hmap_x, mu_y=hmap_y)
    sa.add_heatmap("hmap1", hmap.flatten(), hmap_size, res)
    x1_offset, y1_offset = 2, 0
    x2_offset, y2_offset = 0, 2
    x3_offset, y3_offset = -2, 0
    sa.add_absolute_position_heatmap(
        "1",
        "hmap1",
        x1_offset,
        y1_offset,
        1.0,
    )
    sa.add_absolute_position_heatmap(
        "2",
        "hmap1",
        x2_offset,
        y2_offset,
        1.0,
    )
    sa.add_absolute_position_heatmap(
        "3",
        "hmap1",
        x3_offset,
        y3_offset,
        1.0,
    )

    sa.run()
    s1 = sa.get_rig_instance_pose("1")
    s2 = sa.get_rig_instance_pose("2")
    s3 = sa.get_rig_instance_pose("3")

    assert np.allclose(
        -s1.translation, [x1_offset + hmap_x, y1_offset + hmap_y, 0], atol=res
    )
    assert np.allclose(
        -s2.translation, [x2_offset + hmap_x, y2_offset + hmap_y, 0], atol=res
    )
    assert np.allclose(
        -s3.translation, [x3_offset + hmap_x, y3_offset + hmap_y, 0], atol=res
    )
