"""Test the rig module."""
from typing import cast

import numpy as np
from opensfm import pygeometry, rig, types
from opensfm.dataset import DataSetBase


class MockDataset:
    """Dataset mock class with images and exif data."""

    def __init__(self, exif):
        self.exif = exif

    def images(self):
        return self.exif.keys()

    def load_exif(self, image):
        return self.exif[image]


def metadata(skey, capture_time, lat, lon):
    return {
        "skey": skey,
        "capture_time": capture_time,
        "gps": {"latitude": lat, "longitude": lon},
    }


def test_detect_rigs_two_cams_one_shots() -> None:
    data = MockDataset({"1": metadata(1, 1, 1, 1), "2": metadata(2, 1, 1, 1)})
    rig_info = rig.detect_rigs(data.images(), cast(DataSetBase, data))
    r1 = rig_info["1"]
    r2 = rig_info["2"]
    assert r1["rig"] == r2["rig"]
    assert r1["rig_camera"] != r2["rig_camera"]
    assert r1["rig_pose"] == r2["rig_pose"]


def test_detect_rigs_two_cams_two_shots() -> None:
    data = MockDataset(
        {
            "11": metadata(1, 1, 1, 1),
            "12": metadata(1, 2, 2, 2),
            "21": metadata(2, 1, 1, 1),
            "22": metadata(2, 2, 2, 2),
        }
    )
    rig_info = rig.detect_rigs(data.images(), cast(DataSetBase, data))
    r11 = rig_info["11"]
    r12 = rig_info["12"]
    r21 = rig_info["21"]
    r22 = rig_info["22"]

    assert r11["rig"] == r12["rig"] == r21["rig"] == r22["rig"]
    assert r11["rig_camera"] == r12["rig_camera"]
    assert r21["rig_camera"] == r22["rig_camera"]
    assert r11["rig_camera"] != r21["rig_camera"]
    assert r11["rig_pose"] == r21["rig_pose"]
    assert r12["rig_pose"] == r22["rig_pose"]


def test_pose_mode() -> None:
    poses = [
        pygeometry.Pose([0, 0, 0], [0, 0, 0]),
        pygeometry.Pose([0.1, 0, 0], [0, 0, 0]),
        pygeometry.Pose([0.2, 0, 0], [0, 0, 0]),
        pygeometry.Pose([1, 0, 0], [0, 0, 0]),
    ]
    mode = rig.pose_mode(poses, 0.1, 0.1)
    assert mode == poses[1]

    mode = rig.pose_mode(poses, 1, 1)
    assert mode == poses[2]


def test_create_instances_with_patterns() -> None:
    # A first rig model defined as left/right/top/bottom
    instance1 = [
        "12345_left.jpg",
        "12345_bottom.jpg",
        "12345_top.jpg",
        "12345_right.jpg",
    ]
    instance2 = [
        "1234567_left.jpg",
        "1234567_bottom.jpg",
        "1234567_top.jpg",
        "1234567_right.jpg",
    ]
    patterns_12 = {
        "camera_left": "(left)",
        "camera_right": "(right)",
        "camera_top": "(top)",
        "camera_bottom": "(bottom)",
    }

    # A second one as RED/GREEN/BLUE
    instance3 = [
        "RED_SENSOR_001-1234567.jpg",
        "GREEN_SENSOR_002-1234567.jpg",
        "BLUE_SENSOR_003-1234567.jpg",
    ]
    patterns_3 = {
        "red": "(RED_SENSOR_001)",
        "green": "(GREEN_SENSOR_002)",
        "blue": "(BLUE_SENSOR_003)",
    }

    # Run detection with these two rig model patterns
    rig_patterns = {"rig_model_1": patterns_12, "rig_model_2": patterns_3}
    instances = rig.create_instances_with_patterns(
        instance1 + instance2 + instance3, rig_patterns
    )

    # Ensure we have 2 instance for thr first rig, and one for the second
    assert len(instances) == 2

    rig1 = instances["rig_model_1"]
    assert len(rig1) == 2
    assert [x[0] for x in rig1[0]] == instance1
    assert [x[0] for x in rig1[1]] == instance2

    rig2 = instances["rig_model_2"]
    assert len(rig2) == 1
    assert [x[0] for x in rig2[0]] == instance3


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

    # Compute rig model poses
    rig_model = rig.compute_relative_pose("dummy", pose_instances)

    assert np.allclose(
        [0, -1, 0], rig_model.get_rig_camera("camera_id_1").pose.get_origin(), atol=1e-7
    )
    assert np.allclose(
        [1, 0, 0], rig_model.get_rig_camera("camera_id_2").pose.get_origin(), atol=1e-7
    )
    assert np.allclose(
        [-1, 0, 0], rig_model.get_rig_camera("camera_id_3").pose.get_origin(), atol=1e-7
    )
    assert np.allclose(
        [0, 1, 0], rig_model.get_rig_camera("camera_id_4").pose.get_origin(), atol=1e-7
    )
