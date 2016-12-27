.. Notes and doc on dense matching


Using
=====


Quickstart
----------

An example dataset is available at ``data/berlin``.  You can reconstruct it using by running
::
    bin/run_all data/berlin

This will run the entire SfM pipeline and produce the file ``data/berlin/reconstruction.meshed.json`` as output. To visualize the result you can start a HTTP server running
::
    python -m SimpleHTTPServer

and then browse `<http://localhost:8000/viewer/reconstruction.html#file=/data/berlin/reconstruction.meshed.json>`_
You should see something like

.. image:: images/berlin_viewer.jpg

You can click twice on an image to see it.  Then use arrows to move between images.

If you want to get a denser point cloud, you can run
::
    bin/opensfm undistort data/berlin
    bin/opensfm compute_depthmaps data/berlin

This will run dense multiview stereo matching and produce a denser point cloud stored in ``data/berlin/depthmaps/merged.ply``.  You can visualize that point cloud using MeshLab_ or any other viewer that supports PLY_ files.

For the Berlin dataset you should get something similar to this

.. image:: images/berlin_point_cloud.jpg


To reconstruct your own images,

1. put some images in ``data/DATASET_NAME/images/``, and
2. copy ``data/berlin/config.yml` to ``data/DATASET_NAME/config.yaml``


.. _Meshlab: http://www.meshlab.net/
.. _PLY: http://paulbourke.net/dataformats/ply/


Reconstruction Commands
-----------------------

There are several steps required to do a 3D reconstruction including feature detection, matching, SfM reconstruction and dense matching.  OpenSfM performs these steps using different commands that store the results into files for other commands to use.

The single application ``bin/opensfm`` is used to run those commands.  The first argument of the application is the command to run and the second one is the dataset to run the commands on.

Here is the usage page of ``bin/opensfm``, which lists the available commands
::
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
````````````````
TODO

detect_features
```````````````
TODO

match_features
``````````````
TODO

create_tracks
`````````````
TODO

reconstruct
```````````
TODO

mesh
````
TODO

undistort
`````````
TODO

compute_depthmaps
`````````````````
TODO


Configuration
-------------

TODO explain config.yaml and the available parameters
