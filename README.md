OpenSfM
=======

Structure from Motion library written in Python on top of OpenCV.


Dependencies
------------
* OpenCV
* libmv (included)
* ceres
* jsoncpp
* jhead
* NumPy, networkx, PyYAML

On MacOSX, use

    brew tap homebrew/science
    brew info opencv
    brew install homebrew/science/ceres-solver
    brew tap cuber/homebrew-jsoncpp
    brew install jsoncpp
    brew install jhead
    sudo pip install -r requirement.txt



Building
--------
1. `cd lib; mkdir build; cd build`
2. `cmake ../src`
3. `make`


Running
-------
An example dataset is available at data/berlin.

1. put some images in `data/DATASET_NAME/images/`
2. put config.yaml at `data/DATASET_NAME/config.yaml`
3. run `bin/run_all DATASET_NAME`
4. start a http server with `python -m SimpleHTTPServer`
5. browse `http://localhost:8000/viewer/reconstruction.html#DATASET_NAME/reconstruction.json`
