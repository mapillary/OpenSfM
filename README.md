
labmv
=====

Random tests around SfM and python.

Dependencies
------------
* ceres
* gflags
* glog
* gtest
* OpenCV
* jsoncpp


Building
--------
1. mkdir build; cd build
2. cmake ../src
3. make
4. pray


Running
-------
1. put some images in DATASET_FOLDER/images/
2. copy config.yaml to DATASET_FOLDER/config.yaml
3. run bin/run_all DATASET_FOLDER
4. start a http server with python -m SimpleHTTPServer
5. browse http://localhost:8000/viewer/reconstruction.html#DATASET_FOLDER/reconstruction.json
