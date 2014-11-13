
OpenSfM
=======

Structure from Motion library written in Python on top of OpenCV.

Dependencies
------------
* NumPy
* OpenCV
* ceres
* gflags
* glog
* jsoncpp
* networkx

On MacOSX, use
```
brew install homebrew/science/ceres-solver
brew tap cuber/homebrew-jsoncpp
brew install jsoncpp
sudo pip install -r requirement.txt
```

Building
--------
1. mkdir build; cd build
2. cmake ../src
3. make



Running
-------
1. put some images in DATASET_FOLDER/images/
2. copy config.yaml to DATASET_FOLDER/config.yaml
3. run bin/run_all DATASET_FOLDER
4. start a http server with python -m SimpleHTTPServer
5. browse http://localhost:8000/viewer/reconstruction.html#DATASET_FOLDER/reconstruction.json
