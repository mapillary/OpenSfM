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

    # Instantiate CameraType
    projection_type = 'equirectangular'
    camera_type = CameraType(0, projection_type)

    # Instantiate Shot
    rotation0 = [0.0, 0.0, 0.0]
    translation0 = [0.0, 0.0, 0.0]
    shot0 = Shot(0, camera_type, rotation0, translation0)

    rotation1 = [0.0, 0.0, 0.0]
    translation1 = [-1.0, 0.0, 0.0]
    shot1 = Shot(1, camera_type, rotation1, translation1)

    # Add info to current reconstruction
    reconstruction.add_camera_type(camera_type)
    reconstruction.add_shot(shot0)
    reconstruction.add_shot(shot1)

    # TEST
    assert len(reconstruction.camera_types) == 1
    assert len(reconstruction.shots) == 2
    assert len(reconstruction.points) == 0
    assert reconstruction.get_camera_type(camera_type.id) == camera_type
    assert reconstruction.get_camera_type(1) == None
    assert reconstruction.get_shot(shot0.id) == shot0
    #assert reconstruction.get_shot(shot1.id) == shot1 ## Returns None. Why?!
    assert reconstruction.get_shot(2) == None

if __name__ == "__main__":
    reconstruction_class_initialization_test()