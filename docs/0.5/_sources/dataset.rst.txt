.. Notes and doc on dense matching


Dataset Structure
=================

::

   project/
   ├── config.yaml
   ├── images/
      ├── image_filename
   ├── masks/
      ├── image_filename.png
   ├── gcp_list.txt
   ├── exif/
   ├── camera_models.json
   ├── features/
   ├── matches/
   ├── tracks.csv
   ├── reconstruction.json
   ├── reconstruction.meshed.json
   └── undistorted/
       ├── images/
          ├── image_filename
       ├── masks/
          ├── image_filename.png
       ├── tracks.csv
       ├── reconstruction.json
       └── depthmaps/
           └── merged.ply
    └── stats/
       ├── stats.json
       ├── report.pdf
       ├── topview.png
       ├── matchgraph.png
       ├── heatmap_XXX.png
       ├── residuals_XXX.png

Previous versions of OpenSfM used a different folder structure where undistorted data was not grouped into a single folder.  Please, read and use ``bin/migrate_undistort.sh`` to port old datasets to the new folder structure.


Reconstruction file format
==========================

The main output of OpenSfM is a reconstruction that contains the estimated camera parameters, the estimated camera positions and a sparse set of reconstructed 3D points.  These data are stored in the ``reconstruction.json`` file.  Its format is as follows::

    reconstruction.json: [RECONSTRUCTION, ...]

    RECONSTRUCTION: {
        "cameras": {
            CAMERA_ID: CAMERA,
            ...
        },
        "shots": {
            SHOT_ID: SHOT,
            ...
        },
        "points": {
            POINT_ID: POINT,
            ...
        }
    }

    CAMERA: {
        "projection_type": "perspective",  # Can be perspective, brown, fisheye or equirectangular
        "width": NUMBER,                   # Image width in pixels
        "height": NUMBER,                  # Image height in pixels

        # Depending on the projection type more parameters are stored.
        # These are the parameters of the perspective camera.
        "focal": NUMBER,                   # Estimated focal length
        "k1": NUMBER,                      # Estimated distortion coefficient
        "k2": NUMBER,                      # Estimated distortion coefficient
    }

    SHOT: {
        "camera": CAMERA_ID,
        "rotation": [X, Y, Z],      # Estimated rotation as an angle-axis vector
        "translation": [X, Y, Z],   # Estimated translation
        "gps_position": [X, Y, Z],  # GPS coordinates in the reconstruction reference frame
        "gps_dop": METERS,          # GPS accuracy in meters
        "orientation": NUMBER,      # EXIF orientation tag (can be 1, 3, 6 or 8)
        "capture_time": SECONDS     # Capture time as a UNIX timestamp
    }

    POINT: {
        "coordinates": [X, Y, Z],      # Estimated position of the point
        "color": [R, G, B],            # Color of the point
    }
