.. Notes and doc on dense matching


Using
=====


Quickstart
----------

An example dataset is available at ``data/berlin``.  You can reconstruct it using by running::

    bin/opensfm_run_all data/berlin

This will run the entire SfM pipeline and produce the file ``data/berlin/reconstruction.meshed.json`` as output. To visualize the result you can start a HTTP server running::

    python3 -m http.server

and then browse `<http://localhost:8000/viewer/reconstruction.html#file=/data/berlin/reconstruction.meshed.json>`_
You should see something like

.. image:: images/berlin_viewer.jpg

You can click twice on an image to see it.  Then use arrows to move between images.

If you want to get a denser point cloud, you can run::

    bin/opensfm undistort data/berlin
    bin/opensfm compute_depthmaps data/berlin

This will run dense multiview stereo matching and produce a denser point cloud stored in ``data/berlin/unidtorted/depthmaps/merged.ply``.  You can visualize that point cloud using MeshLab_ or any other viewer that supports PLY_ files.

For the Berlin dataset you should get something similar to this

.. image:: images/berlin_point_cloud.jpg


To reconstruct your own images,

1. put some images in ``data/DATASET_NAME/images/``, and
2. copy ``data/berlin/config.yaml` to ``data/DATASET_NAME/config.yaml``


.. _Meshlab: http://www.meshlab.net/
.. _PLY: http://paulbourke.net/dataformats/ply/


Reconstruction Commands
-----------------------

There are several steps required to do a 3D reconstruction including feature detection, matching, SfM reconstruction and dense matching.  OpenSfM performs these steps using different commands that store the results into files for other commands to use.

The single application ``bin/opensfm`` is used to run those commands.  The first argument of the application is the command to run and the second one is the dataset to run the commands on.

Here is the usage page of ``bin/opensfm``, which lists the available commands::

    usage: opensfm [-h] command ...

    positional arguments:
      command            Command to run
        extract_metadata
                         Extract metadata form images' EXIF tag
        detect_features  Compute features for all images
        match_features   Match features between image pairs
        create_tracks    Link matches pair-wise matches into tracks
        reconstruct      Compute the reconstruction
        mesh             Add delaunay meshes to the reconstruction
        undistort        Save radially undistorted images
        compute_depthmaps
                         Compute depthmap
        export_ply       Export reconstruction to PLY format
        export_openmvs   Export reconstruction to openMVS format
        export_visualsfm
                         Export reconstruction to NVM_V3 format from VisualSfM

    optional arguments:
      -h, --help         show this help message and exit


extract_metadata
~~~~~~~~~~~~~~~~

This commands extracts EXIF metadata from the images an stores them in the ``exif`` folder and the ``camera_models.json`` file.

The following data is extracted for each image:

- ``width`` and ``height``: image size in pixels

- ``gps`` ``latitude``, ``longitude``, ``altitude`` and ``dop``: The GPS coordinates of the camera at capture time and the corresponding Dilution Of Precision).  This is used to geolocate the reconstruction.

- ``capture_time``: The capture time. Used to choose candidate matching images when the option ``matching_time_neighbors`` is set.

- ``camera orientation``: The EXIF orientation tag (see this `exif orientation documentation`_).  Used to orient the reconstruction straight up.

- ``projection_type``: The camera projection type.  It is extracted from the GPano_ metadata and used to determine which projection to use for each camera.  Supported types are `perspective`, `equirectangular` and `fisheye`.

- ``focal_ratio``: The focal length provided by the EXIF metadata divided by the sensor width. This is used as initialization and prior for the camera focal length parameter.

- ``make`` and ``model``: The camera make and model.  Used to build the camera ID.

- ``camera``: The camera ID string. Used to identify a camera.  When multiple images have the same camera ID string, they will be assumed to be taken with the same camera and will share its parameters.


Once the metadata for all images has been extracted, a list of camera models is created and stored in ``camera_models.json``.  A camera model is created for each diferent camera ID string found on the images.

For each camera ID, the cammera model parameters are chosen using the following procedure.

- If the camera ID exists in the ``camera_models_overrides.json`` then the parameters are taken from that file. 
- Otherwise, if the camera ID exists in an internal calibration database, then the camera parameters are taken from the database.
- Otherwise, the camera parameters are inferred from the avalable EXIF metadata.


Providing additional metadata
'''''''''''''''''''''''''''''

When some metadata is missing or erroneous in the EXIF, it can be provided on the ``exif_overrides.json`` file.  This file must contain a dictionary mapping image names to the metadata fields that will override the values in the EXIF.

For example, to set the GPS location of an image that might not have it available in its EXIF tags, we will write the following in the ``exif_overrides.json`` file::

    {
        "image_name.jpg": {
            "gps": {
                "latitude": 52.51891, 
                "longitude": 13.40029,
                "altitude": 27.0, 
                "dop": 5.0
            }
        }
    }

These values are used during the ``extract_metadata``, so we will need to rerun that command after writing the file.


Providing your own camera parameters
''''''''''''''''''''''''''''''''''''

By default, the camera parameters are taken from the EXIF metadata using the procedure described above.  If you know the camera parameters that you want to use, you can put them in a file named ``camera_models_overrides.json`` in the project folder.  This file should have the same structure as ``camera_models.json``.  That is a dictionary mapping camera identifiers to the desired camera parameters.

When running the ``extract_metadata`` command, the parameters of any camera present in the ``camera_models_overrides.json`` file will be copied to ``camera_models.json`` overriding the default ones.

The simplest way to create the ``camera_models_overrides.json`` file is to rename ``camera_models.json`` and modify the parameters.  You will need to rerun the ``extract_metadata`` command after that.

Optionally, you may want to override the camera parameters of **all** cameras in the dataset.  To do so, use ``"all"`` as camera ID.  For example::

    {
        "all": {
            "projection_type": "perspective",
            "width": 1920,
            "height": 1080,
            "focal": 0.9,
            "k1": 0.0,
            "k2": 0.0,
        }
    }

will set all cameras to use a perspective projection model.  Likewise::

    {
        "all": {
            "projection_type": "fisheye",
            "width": 1920,
            "height": 1080,
            "focal": 0.5,
            "k1": 0.0,
            "k2": 0.0,
        }
    }

will set all cameras to use a fisheye projection model.  And::

    {
        "all": {
            "projection_type": "equirectangular",
            "width": 2000,
            "height": 1000,
        }
    }

will set all cameras to use an equirectangular panoramic projection model.

Have a look at the `spherical 360 images dataset`_ for a complete example using ``camera_models_overrides.json`` to specify that the camera is taking 360 equirectangular images.


.. _`exif orientation documentation`: http://sylvana.net/jpegcrop/exif_orientation.html
.. _GPano: https://developers.google.com/streetview/spherical-metadata
.. _`spherical 360 images dataset`: https://www.dropbox.com/sh/3vabbmrhqqbagp5/AABi14O2tWMbxAX91jaaQY77a?dl=0

detect_features
~~~~~~~~~~~~~~~
This command detects feature points in the images and stores them in the `feature` folder.


match_features
~~~~~~~~~~~~~~
This command matches feature points between images and stores them in the `matches` folder.  It first determines the list of image pairs to run, and then run the matching process for each pair to find corresponding feature points.

Since there are a lot of possible image pairs, the process can be very slow.  It can be speeded up by restricting the list of pairs to match.  The pairs can be restricted by GPS distance, capture time or file name order.


create_tracks
~~~~~~~~~~~~~
This command links the matches between pairs of images to build feature point tracks.  The tracks are stored in the `tracks.csv` file.  A track is a set of feature points from different images that have been recognized to correspond to the same pysical point.


reconstruct
~~~~~~~~~~~
This command runs the incremental reconstruction process.  The goal of the reconstruction process is to find the 3D position of tracks (the `structure`) together with the position of the cameras (the `motion`).  The computed reconstruction is stored in the ``reconstruction.json`` file.


mesh
~~~~
This process computes a rough triangular mesh of the scene seen by each images.  Such mesh is used for simulating smooth motions between images in the web viewer.  The reconstruction with the mesh added is stored in ``reconstruction.meshed.json`` file.

Note that the only difference between ``reconstruction.json`` and ``reconstruction.meshed.json`` is that the later contains the triangular meshes.  If you don't need that, you only need the former file and there's no need to run this command.


undistort
~~~~~~~~~
This command creates undistorted version of the reconstruction, tracks and images.  The undistorted version can later be used for computing depth maps.


compute_depthmaps
~~~~~~~~~~~~~~~~~
This commands computes a dense point cloud of the scene by computing and merging depthmaps.  It requires an undistorted reconstructions.  The resulting depthmaps are stored in the ``depthmaps`` folder and the merged point cloud is stored in ``undistorted/depthmaps/merged.ply``


Configuration
-------------

SfM algorithms have options and depend on various parameters.  OpenSfM comes setup with default values for each option but you might want to tune some options for a particular dataset.  Options used to reconstruct a dataset can be set by editing the file ``DATASET_PATH/config.yaml``.  Any option present in this file will override the default.

Checkout `the default configuration <_modules/opensfm/config.html>`_ to see the list of options.


.. include:: gcp.rst
