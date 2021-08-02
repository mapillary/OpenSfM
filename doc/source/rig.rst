.. Doc on rig

.. _rig-model:

Rig Models
==========


Coordinate Systems
------------------

Rig are defined by a fixed assembly of cameras that are triggered at the same instant.
The following terms define such assembly and capture in OpenSfM terminology :

 - A `RigCamera` is a camera of the rig assembly defined as a combination of an existing camera model (it refers only to its ID) and its pose wrt. the rig assembly coordinate frame. `RigCamera` are defined in the `rig_cameras.json` as the following::

 {
    "RIG_CAMERA_ID":
    {
        "translation": translation of the rig frame wrt. the RigCamera frame
        "rotation": rotation bringing a point from rig frame to the RigCamera frame
        "camera": camera model ID of this RigCamera
    },
    ...

 - A `RigInstance` is a list of `Shots`, each of which correspond to a `RigCamera` of the `RigModel` and the actual pose of the `RigModel` in the world : it's indeed an instanciation of the `RigModel` by combining `Shots`. These instances are defined in the `rig_assignments.json` file as follows::

    {
        [
            [
                "FILENAME",
                "RIG_CAMERA_ID1"
            ],
            [
                "FILENAME",
                "RIG_CAMERA_ID2"
            ],
            ...
            [
                "FILENAME",
                "RIG_CAMERA_IDn"
            ]
        ],
        [
            [
                "FILENAME",
                "RIG_CAMERA_ID1"
            ],
            [
                "FILENAME",
                "RIG_CAMERA_ID2"
            ],
            ...
            [
                "FILENAME",
                "RIG_CAMERA_IDn"
            ]
        ],
        ...



A picture is often worth many words :
|rig_frame|

Usage
~~~~~

Given the above, one can either define manually the `rig_assignments.json` and the `rig_cameras.json`, or use
the OpenSfM `create_rig` command. This commands will take a JSON string as input to help it defines the rig
instances based on the filenames, such as::

    {
        "RIG_CAMERA_ID1": "PATTERN1",
        "RIG_CAMERA_ID2": "PATTERN2",
        ...


Where "PATTERN" is the following :
 - A camera model ID if the method for assigning is `camera`. A given camera model ID correspond to one `RigCamera`
 - A REGEX with the form (.*) where the part in parenthesis identifies the camera models, when the method is `pattern`. For example, it would be "(RED)" or "(GREEN)" for multispectral data.

 Based on this instances, it then run SfM on a small subset on the data and infers some averaged rig cameras poses, which are then written to `rig_cameras.json`.

 .. |rig_frame| image:: images/rig_frame.png
