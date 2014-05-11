
labmv
=====

Random tests around SfM and python.

Dependencies
------------
* libmv
* pyopencv
* jansson

Building
--------
1. build libs in the third_party folder
2. build the code in src
    2.1. mkdir build; cd build
    2.2. cmake ../src
    2.3. make
    2.4. pray

Running
-------
1. put some images in DATASET_FOLDER/images/
2. run bin/run_all DATASET_FOLDER
3. start a http server with python -m SimpleHTTPServer
4. browse http://localhost:8000/viewer/reconstruction.html#DATASET_FOLDER/reconstruction.json
