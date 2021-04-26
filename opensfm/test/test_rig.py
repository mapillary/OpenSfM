"""Test the rig module."""
import numpy as np
from opensfm import pygeometry, rig, types


def test_create_instances_with_patterns() -> None:
    # A first rig model defined as left/right/top/bottom

    # A complete instance
    instance1 = [
        "12345_left.jpg",
        "12345_bottom.jpg",
        "12345_top.jpg",
        "12345_right.jpg",
    ]

    # An incomplete one
    instance2 = [
        "1234567_left.jpg",
        "1234567_bottom.jpg",
        "1234567_top.jpg",
    ]
    patterns_12 = {
        "camera_left": "(left)",
        "camera_right": "(right)",
        "camera_top": "(top)",
        "camera_bottom": "(bottom)",
    }

    # A second one as RED/GREEN/BLUE
    instance3 = [
        "RED_SENSOR_001-12345678.jpg",
        "GREEN_SENSOR_002-12345678.jpg",
        "BLUE_SENSOR_003-12345678.jpg",
    ]
    patterns_3 = {
        "red": "(RED_SENSOR_001)",
        "green": "(GREEN_SENSOR_002)",
        "blue": "(BLUE_SENSOR_003)",
    }

    # Two single shots
    instance4 = [
        "RED_toto.jpg",
        "tata.jpg",
    ]

    # Run detection with these two rig model patterns
    rig_patterns = patterns_12
    rig_patterns.update(patterns_3)
    instances, single_shots = rig.create_instances_with_patterns(
        instance1 + instance2 + instance3 + instance4, rig_patterns
    )

    # Ensure we have 2 instance for the first rig, and 1 for the second
    assert len(instances) == 3

    # Ensure the two single shots
    assert len(single_shots) == 2

    recovered_instance1 = instances["12345_.jpg"]
    assert [x[0] for x in recovered_instance1] == instance1

    recovered_instance2 = instances["1234567_.jpg"]
    assert [x[0] for x in recovered_instance2] == instance2

    recovered_instance3 = instances["-12345678.jpg"]
    assert [x[0] for x in recovered_instance3] == instance3


def test_compute_relative_pose() -> None:
    # 4-cameras rig
    camera1 = pygeometry.Camera.create_spherical()
    camera1.id = "camera1"
    camera2 = pygeometry.Camera.create_spherical()
    camera2.id = "camera2"
    camera3 = pygeometry.Camera.create_spherical()
    camera3.id = "camera3"
    camera4 = pygeometry.Camera.create_spherical()
    camera4.id = "camera4"

    # a bit cumbersome that we need to have some reconstruction
    rec = types.Reconstruction()
    rec.add_camera(camera1)
    rec.add_camera(camera2)
    rec.add_camera(camera3)
    rec.add_camera(camera4)

    # First rig instance
    rec.create_shot("shot1", "camera1", pygeometry.Pose([0, 0, 0], [-2, -2, 0]))
    rec.create_shot("shot2", "camera2", pygeometry.Pose([0, 0, 0], [-3, -3, 0]))
    rec.create_shot("shot3", "camera3", pygeometry.Pose([0, 0, 0], [-1, -3, 0]))
    rec.create_shot("shot4", "camera4", pygeometry.Pose([0, 0, 0], [-2, -4, 0]))

    # Second rig instance (rotated by pi/2 around Z)
    pose_instance = pygeometry.Pose([0, 0, -1.5707963])
    pose_instance.set_origin([-6, 0, 0])
    rec.create_shot("shot5", "camera1", pose_instance)
    pose_instance.set_origin([-7, 1, 0])
    rec.create_shot("shot6", "camera2", pose_instance)
    pose_instance.set_origin([-7, -1, 0])
    rec.create_shot("shot7", "camera3", pose_instance)
    pose_instance.set_origin([-8, 0, 0])
    rec.create_shot("shot8", "camera4", pose_instance)

    pose_instances = [
        [
            (
                rec.shots["shot1"],
                "camera_id_1",
            ),
            (
                rec.shots["shot2"],
                "camera_id_2",
            ),
            (
                rec.shots["shot3"],
                "camera_id_3",
            ),
            (
                rec.shots["shot4"],
                "camera_id_4",
            ),
        ],
        [
            (
                rec.shots["shot5"],
                "camera_id_1",
            ),
            (
                rec.shots["shot6"],
                "camera_id_2",
            ),
            (
                rec.shots["shot7"],
                "camera_id_3",
            ),
            (
                rec.shots["shot8"],
                "camera_id_4",
            ),
        ],
    ]

    # Compute rig cameras poses
    rig_cameras = rig.compute_relative_pose(pose_instances)

    assert np.allclose(
        [0, -1, 0], rig_cameras["camera_id_1"].pose.get_origin(), atol=1e-7
    )
    assert np.allclose(
        [1, 0, 0], rig_cameras["camera_id_2"].pose.get_origin(), atol=1e-7
    )
    assert np.allclose(
        [-1, 0, 0], rig_cameras["camera_id_3"].pose.get_origin(), atol=1e-7
    )
    assert np.allclose(
        [0, 1, 0], rig_cameras["camera_id_4"].pose.get_origin(), atol=1e-7
    )
