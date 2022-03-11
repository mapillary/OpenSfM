.. Doc on rig

.. _sensors_database:

Calibration Database
====================


Overview
--------

In order to produce accurate geometry, structure-from-motion (SfM) needs to have correct estimates of the imaging sensor geometry, such as : lens type (fisheye, perspective, spherical), focal, distorsion, principal point. Please refer to the `Geometric Models`_ section for a comprehensive list of camera internal parameters (calibration).

While reconstructing the scene (using incremental SfM), OpenSfM will adjust for the camera calibration values that best explain the seen geometry. However, in order to get optimal and failsafe results, it is recommended to have a first good guess of the calibration values. By default, OpenSfM will try to get these values by reading the image EXIFs, where the focal length can be red, and is one of the most important of the calibration values. However, sometimes, EXIFs does not contain such value, or it is erroneous, and/or it is better to have other values than just the focal length.

Here comes sensors databases to the rescue. These are files stored under ``opensfm/data`` :
 - ``sensor_data_detailed.json``
 - ``sensor_data.json``
 - ``sensor_calibration.yaml``

sensor_data_detailed.json
-------------------------

This file contains physical sensor's width and height, in millimeters, for a given ``model make`` sensor (see `extract_metadata`_). It means that if only the focal length is available in the EXIFs, since we also have the sensor physical size, we know the full sensor geometry.


sensor_data.json
----------------

This file contains a multiplicative factor for a given ``model make`` sensor (see `extract_metadata`_). When applied to the EXIFs focal length, this factor gives the focal 35mm equivalent. Since we know the dimensions of 35mm equivalent (24x32 mm), we again know the full sensor geometry.

camera_calibration.json
------------------------

This file contains the full definition (in OpenSfM format) of camera calibrations. Calibration are for a given ``make`` (see `extract_metadata`_), and then, they're further refined :
 - If ``ALL`` is specified, then the calibration is valid for all ``make model`` camera independant of their ``model`` value
 - If ``MODEL`` is specified, then calibrations are per actual ``model``
 - If ``FOCAL`` is specified, then calibrations are per focal length red from the EXIFs
