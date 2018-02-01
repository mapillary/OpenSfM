.. Doc on the split/merge pipeline for large datasets


Splitting a large dataset into smaller submodels
================================================

Large datasets can be slow to process.  An option to speed up the reconstruction process is to split them into smaller datasets.  We will call each of the small datasets a *submodel*.  Smaller datasets run faster because they involve fewer images on each bundle adjustment iteration.  Additionally, the reconstruction of the different submodels can be done in parallel.

Since the reconstructions of the submodels are done independently, they will not be necessarily aligned with each other.  Only the GPS positions of the images and the ground control points will determine the alignment.  When the neighboring reconstructions share cameras or points, it is possible to enforce the alignment of common cameras and points between the different reconstructions.

In the following, we describe the commands that help to split a large dataset and aligning the resulting submodels.


Creating submodels
------------------

The command ``create_submodels`` splits a dataset into submodels.  The splitting is done based on the GPS position of the images.  Therefore, it is required to run ``extract_metadata`` before so that the GPS positions are read from the image metadata.

Additionally, the feature extraction and matching can also be done before creating the submodels.  This makes it possible for each submodel to reuse the features and matches of the common images.

The process to split a dataset into submodels is then::

    bin/opensfm extract_metadata path/to/dataset
    bin/opensfm detect_features path/to/dataset
    bin/opensfm match_features path/to/dataset
    bin/opensfm create_submodels path/to/dataset

Submodels dataset structure
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The submodels are created inside the ``submodels`` folder.  Each submodel folder is a valid OpenSfM dataset.  The images, EXIF metadata, features, and matches are shared with the global dataset by using symbolic links.

::

    project/
    ├── images/
    ├── opensfm/
    ├── image_list.txt
    ├── image_list_with_gps.csv    # list of original images with GPS position
    ├── exif
    ├── features                   # eventually
    ├── matches                    # eventually
    └── submodels/
        ├── clusters_with_neighbors.geojson  # geojson file with all images as features with corresponding submodel as a property
        ├── clusters_with_neighbors.npz
        ├── clusters.npz
        ├── image_list_with_gps.tsv
        ├── submodel_0000/
            │   ├── image_list.txt        # images of submodel_0000
            │   ├── config.yaml           # copy from global equivalent
            │   ├── images/               # link to global equivalent
            │   ├── exif/                 # link to global equivalent
            │   ├── features/             # link to global equivalent
            │   ├── matches/              # link to global equivalent
            │   ├── camera_models.json    # link to global equivalent
            │   └── reference_lla.json    # link to global equivalent
            └── submodel_0001/
        └── ...

Config parameters
~~~~~~~~~~~~~~~~~

The creation of the submodels can be tuned by different parameters.

There are two parameters controlling the size and overlap of the submodels.  The parameters need to be adjusted to the size.

- ``submodel_size``
  Average number of images per submodel.  The splitting of the dataset is done by clustering image locations into groups.  K-means clustering is used and ``k`` is set to be the number of images divided by ``submodel_size``.

- ``submodel_overlap``
  Radius of the overlapping region between submodels in meters.  To be able to align the different submodels, there needs to be some common images between the neighboring submodels.  Any image that is closer to a cluster than ``submodel_overlap`` it is added to that cluster.


The folder structure of the submodels can also be controlled using the following parameters. You shouldn't need to do change them.

- ``submodels_relpath``
  Relative path to the submodels directory.  Cluster information will be stored in this directory.

- ``submodel_relpath_template``
  Template to generate the relative path to a submodel directory.

- ``submodel_images_relpath_template``
  Template to generate the relative path to a submodel images directory.

- ``submodel_use_symlinks``
  When true, global features and matches will be symlinked in each submodel so that they can be reused.  When false, features and matches will need to be run for each submodel.

Providing the image groups
~~~~~~~~~~~~~~~~~~~~~~~~~~
The ``create_submodels`` command clusters images into groups to decide the partition into submodels.  If you already know how you want to split the dataset, you can provide that information and it will be used instead of the clustering algorithm.

The grouping can be provided by adding a file named ``image_groups.txt`` in the main dataset folder.  The file should have one line per image.  Each line should have two words: first the name of the image and second the name of the group it belongs to.  For example::

    01.jpg A
    02.jpg A
    03.jpg B
    04.jpg B
    05.jpg C

will create 3 submodels.

Starting from this groups, ``create_submodels`` will add to each submodel the images in the overlap area based on the ``submodels_overlap`` parameter.


Running the reconstruction for each submodel
--------------------------------------------

Since each submodel is a valid OpenSfM dataset, the reconstruction can be run using the standard commands.  Assuming features and matches have already been computed, we will need to run::

    bin/opensfm create_tracks path/to/dataset/submodels/submodel_XXXX
    bin/opensfm reconstruct path/to/dataset/submodels/submodel_XXXX

for each submodel.  This can be run in parallel since the submodels are independent.


Aligning submodels
------------------

Once every submodel has a reconstruction, they can be aligned by using the command::

    bin/opensfm align_submodels path/to/dataset

This command will load all the reconstructions, look for cameras and points shared between the reconstructions, and move each reconstruction rigidly in order best align the corresponding cameras and points.
