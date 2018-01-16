.. Overview of the incremental reconstruction algorithm


Incremental reconstruction algorithm
=====================================

OpenSfM implements an incremental structure from motion algorithm.  This is reconstruction algorithm that starts building a reconstruction of a single image pair and then iteratively add the other images to the reconstruction one at a time.

The algorithm is implemented in the ``reconstruction.py`` module and the main entry point is the :func:`~opensfm.reconstruction.incremental_reconstruction` function.

The algorithm has three main steps:

1. Find good initial pairs
2. Bootstrap the reconstruction with two images
3. Grow the reconstruction by adding images one at a time

If after step 3 there are images that have not yet been reconstructed, steps 2 and 3 are repeated to generate more reconstructions.


1. Finding good initial pairs
-----------------------------

To compute the initial reconstruction using two images, there needs to be enough parallax between them.  That is, the camera should have been displaced between the two shots, and the displacement needs to be large enough compared to the distance to the scene.

To compute whether there is enough parallax, we start by trying to fit a rotation only camera model to the two images.  We only consider image pairs that have a significant portion of the correspondences that can not be explained by the rotation model.  We compute the number of outliers of the model and accept it only if the portion of outliers is larger than 30%.

The accepted image pairs are sorted by the number of outliers of the rotation only model.

This step is done by the :func:`~opensfm.reconstruction.compute_image_pairs` function.


2. Boostraping the reconstruction
---------------------------------

To bootstrap the reconstruction, we use the first image pair.  If initialization fails we try with the next on the list.  If the initialization works, we pass it to the next step to grow it with more images.

The reconstruction from two views can be done by two algorithms depending on the geometry of the scene.  If the scene is flat, a plane-based initialization is used, if it is not flat, then the five-point algorithm is used.  Since we do not know a priori if the scene is flat, both initializations are computed and the one that produces more points is retained (see the :func:`~opensfm.reconstruction.two_view_reconstruction_general` function).

If the pair gives enough inliers we initialize a reconstruction with the corresponding poses, triangulate the matches and bundle adjust it.


3. Growing the reconstruction
-----------------------------

Given the initial reconstruction with two images, more images are added one by one starting with the one that sees more of the reconstructed points.

To add an image it needs first needs to be aligned to the reconstruction.  This is done by finding the camera position that makes the reconstructed 3D points project to the corresponding position in the new image.  The process is called resectioning and is done by the :func:`~opensfm.reconstruction.resect` function.

If resectioning works, the image is added to the reconstruction. After adding it, all features of the new image that are also seen in other reconstructed images are triangulated.  If needed, the reconstruction is then bundle adjusted and eventually all features are re-triangulated.  The parameters ``bundle_interval``, ``bundle_new_points_ratio``, ``retriangulation`` and ``retriangulation_ratio`` control when bundle and re-triangulation are needed.

Finally, if the GPS positions of the shots or Ground Control Points (GPS) are available, the reconstruction is rigidly moved to best align to those.
