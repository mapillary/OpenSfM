
Incremental reconstruction algorithm
=====================================

The following is the product of painstaking reverse-engineering from the ``reconstruction`` module's source code.

It is a high-level, natural-language version of the ``incremental_reconstruction`` function and its sub-units.

Incremental reconstruction
--------------------------

Obtain image pairs with > 30% outliers (via OpenCV's
``findHomography()`` with ``RANSAC``), sorted by number of common
tracks. (``compute_image_pairs``)

For each pair ``(img1, img2)`` remaining:

1. Initiate new reconstruction on pair. (``bootstrap_reconstruction``)
2. Grow reconstruction as applicable, taking from image list without
   replacement. (``grow_reconstruction``)

Initiating a new reconstruction
-------------------------------

Obtain relative pose of camera 2 from camera 1's POV
(``two_view_reconstruction``).

If <= 5 inliers from this step, abort by returning early. Otherwise:

Form the snapshot objects: ``shot1`` with default pose, ``shot2`` from
established relative pose.

Triangulate features in ``img1`` and add them to the reconstruction.

Provided more than ``five_point_algo_min_inliers`` (50) points were
added:

-  Bundle adjust ``shot2``.
-  Re-triangulate tracks.
-  Bundle adjust ``shot2`` again.

Initial "bootstrapped" reconstruction completed.

two_view_reconstruction
```````````````````````````

Estimate relative pose on each camera's POV of its own points (pixel
bearings). (OpenGV's ``relative_pose_ransac`` with ``"STEWENIUS"``)

Use this pose estimate to triangulate the pixel bearings into two sets
of 3D positions.

Normalise the bearings, effectively projecting them back to the image
planes.

Considering both sets from the respective camera coordinate systems:
compute distance between original bearings and re-projected ones.

Remove outliers, defined as points with a reprojection error of at least
``five_point_algo_threshold`` (0.006).

Run through OpenGV's ``relative_pose_optimize_nonlinear``, yielding
refined pose (beyond that, no idea what it does. Some nonlinear
optimisation in Ceres solver.)

Reproject, compare and filter outliers again.

Return the number of inliers along with the relative pose of camera 2
wrt camera 1.

Grow reconstruction
-------------------

Bundle adjust entire reconstruction (not an individual shot).

Align reconstruction to GPS, based on GCPs (Ground Control Points).

(*) Obtain images (not pairs), from highest number of tracks to least.

For each image: attempt to ``resect`` (whatever that means?
``absolute_pose_ransac`` with ``"KNEIP"``, add shot to reconstruction,
bundle adjust shot.)

If couldn't ``resect``, skip ahead to try the next image. Once we find
an image that can be successfully resected:

Shot was added to reconstruction during resection, so remove it from the
list of remaining images in ``incremental_reconstruction``.

Triangulate features in shot, based on pose information from the
``resect`` step (presumably.)

Decide whether to bundle adjust. Bundle adjustment happens every
``bundle_interval`` (0) snapshots resected and added. It also happens
after growing the number of points in the reconstruction by a factor of
``bundle_new_points_ratio`` (1.2).

If it's time to bundle adjust:

-  Bundle adjust the entire reconstruction (without GCP data).
-  Get rid of all tracks with reprojection error larger than
   ``bundle_outlier_threshold`` (0.008).
-  Align reconstruction again, using GPS and GCPs.

Decide whether to re-triangulate. This can be disabled entirely
(``retriangulation`` config parameter). If active, happens after growing
the number of points in the reconstruction by a factor of
``retriangulation_ratio`` (1.25).

If it's time to re-triangulate:

-  Re-triangulate all tracks in reconstruction.
-  Bundle adjust entire reconstruction (again, without GCP data).

Since resection succeeded, go back to (*) to re-generate the image list
and search for another candidate image to resect into a snapshot.

Once (*) fails:

Bundle adjust entire reconstruction - this time, using GCPs.

Align reconstruction using GPS and GCPs.

Finished.