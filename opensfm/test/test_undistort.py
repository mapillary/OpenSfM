import itertools

import numpy as np
from opensfm import undistort, pygeometry, types


def test_perspective_views_of_a_panorama():
    reconstruction = types.Reconstruction()
    camera = pygeometry.Camera.create_spherical()
    camera.id = "spherical_camera"
    camera.width = 8000
    camera.height = 4000
    reconstruction.add_camera(camera)
    pose = pygeometry.Pose([1, 2, 3], [4, 5, 6])
    spherical_shot = reconstruction.create_shot("shot1", camera.id, pose=pose)

    urec = types.Reconstruction()
    rig_instance_count = itertools.count()
    undistort.perspective_views_of_a_panorama(
        spherical_shot, 800, urec, "jpg", rig_instance_count
    )

    assert len(urec.rig_cameras) == 6
    assert len(urec.rig_instances) == 1
    assert len(urec.rig_instances[0].shots) == 6
    front_found = False
    for shot in urec.rig_instances[0].shots.values():
        assert np.allclose(shot.pose.get_origin(), spherical_shot.pose.get_origin())
        if shot.rig_camera_id == "front":
            front_found = True
            assert np.allclose(shot.pose.rotation, spherical_shot.pose.rotation)
        else:
            assert not np.allclose(shot.pose.rotation, spherical_shot.pose.rotation)
    assert front_found
