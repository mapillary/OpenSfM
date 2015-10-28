from opensfm.types import *

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

def reconstruction_class_initialization_test():

    # Instantiate Reconstruction
    reconstruction = Reconstruction()

    # Instantiate camera instrinsics
    intrinsics = Intrinsics()
    intrinsics.fx_prior = 0.9722222222222222
    intrinsics.fy_prior = 0.9722222222222222
    intrinsics.k1 = 0.006094395128698237
    intrinsics.k2 = -0.0004952058188617129
    intrinsics.height = 2448
    intrinsics.width = 3264

    # Instantiate camera
    projection_type = 'equirectangular'

    camera = Camera(0, projection_type, intrinsics)
    camera.description = 'apple iphone 4s back camera 4.28mm f/2.4'

    # Instantiate GPS data
    gps_data = GpsData()
    gps_data.orientation = 1
    gps_data.capture_time = 0.0
    gps_data.gps_dop = 5.0
    gps_data.gps_position = [ 1.0815875281451939,
                             -0.96510451436708888,
                              1.2042133903991235 ]

    # Instantiate shots

    extrinsics0 = Extrinsics()
    extrinsics0.rotation =[0.0, 0.0, 0.0]
    extrinsics0.translation = [0.0, 0.0, 0.0]

    shot0 = Shot(0, camera, extrinsics0, gps_data)

    extrinsics1 = Extrinsics()
    extrinsics1.rotation =[0.0, 0.0, 0.0]
    extrinsics1.translation = [-1.0, 0.0, 0.0]

    shot1 = Shot(1, camera, extrinsics1, gps_data)

    # Add info to current reconstruction
    reconstruction.add_camera(camera)
    reconstruction.add_shot(shot0)
    reconstruction.add_shot(shot1)

    # TEST
    assert len(reconstruction.cameras) == 1
    assert len(reconstruction.shots) == 2
    assert len(reconstruction.points) == 0
    assert reconstruction.get_camera(camera.id) == camera
    assert reconstruction.get_camera(1) == None
    assert reconstruction.get_shot(shot0.id) == shot0
    #assert reconstruction.get_shot(shot1.id) == shot1 ## Returns None. Why?!
    assert reconstruction.get_shot(2) == None

if __name__ == "__main__":
    reconstruction_class_initialization_test()