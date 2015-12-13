from opensfm import types

"""
Trying to imitate the following structure

reconstruction = {
    "cameras": {
        "theta": {
            "projection_type": "equirectangular"
        }
    },

    "shots" : {
        'im1': {
            "camera": "theta",
            "rotation": [0.0, 0.0, 0.0],
            "translation": [0.0, 0.0, 0.0],
        },
        'im2': {
            "camera": "theta",
            "rotation": [0, 0, 0.0],
            "translation": [-1, 0, 0.0],
        },
    },

    "points" : {
    },
}
"""


def test_reconstruction_class_initialization():

    # Instantiate Reconstruction
    reconstruction = types.Reconstruction()

    # Instantiate camera instrinsics
    intrinsics = types.Intrinsics()
    intrinsics.fx_prior = 0.9722222222222222
    intrinsics.fy_prior = 0.9722222222222222
    intrinsics.k1 = 0.006094395128698237
    intrinsics.k2 = -0.0004952058188617129
    intrinsics.height = 2448
    intrinsics.width = 3264

    # Instantiate camera

    camera = types.Camera()
    camera.id = 'apple iphone 4s back camera 4.28mm f/2.4'
    camera.projection_type = 'equirectangular'
    camera.intrinsics = intrinsics

    # Instantiate GPS data
    gps_data = types.GpsData()
    gps_data.orientation = 1
    gps_data.capture_time = 0.0
    gps_data.gps_dop = 5.0
    gps_data.gps_position = [1.0815875281451939,
                             -0.96510451436708888,
                             1.2042133903991235]

    # Instantiate shots
    pose0 = types.Pose()
    pose0.rotation = [0.0, 0.0, 0.0]
    pose0.translation = [0.0, 0.0, 0.0]

    shot0 = types.Shot()
    shot0.id = 0
    shot0.camera = camera
    shot0.pose = pose0
    shot0.gps_data = gps_data

    pose1 = types.Pose()
    pose1.rotation = [0.0, 0.0, 0.0]
    pose1.translation = [-1.0, 0.0, 0.0]

    shot1 = types.Shot()
    shot1.id = 1
    shot1.camera = camera
    shot1.pose = pose1
    shot1.gps_data = gps_data

    # Add info to current reconstruction
    reconstruction.add_camera(camera)
    reconstruction.add_shot(shot0)
    reconstruction.add_shot(shot1)

    # TEST
    assert len(reconstruction.cameras) == 1
    assert len(reconstruction.shots) == 2
    assert len(reconstruction.points) == 0
    assert reconstruction.get_camera(camera.id) == camera
    assert reconstruction.get_camera(1) is None
    assert reconstruction.get_shot(shot0.id) == shot0
    assert reconstruction.get_shot(shot1.id) == shot1
    assert reconstruction.get_shot(2) is None


if __name__ == "__main__":
    test_reconstruction_class_initialization()
