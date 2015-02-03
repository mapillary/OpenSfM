OpenSfM
=======

Structure from Motion library written in Python on top of OpenCV.

## Dependencies

* [OpenCV][]
* [Ceres Solver][]
* [boost-python][]
* [NumPy][], [SciPy][], networkx, PyYAML, exifread

### Installing dependencies on MacOSX

Use

    brew tap homebrew/science
    brew install opencv
    brew install homebrew/science/ceres-solver
    brew install boost
    sudo pip install -r requirements.txt

Be sure to update your `PYTHONPATH` to include `/usr/local/lib/python2.7/site-packages` where OpenCV has been installed:

    export PYTHONPATH=/usr/local/lib/python2.7/site-packages:$PYTHONPATH


### Installing dependencies on Ubuntu

 1. [OpenCV][] - Install by following the steps in the Ubuntu OpenCV  [installation guide](https://help.ubuntu.com/community/OpenCV). An alternative instruction tested for Ubuntu 10.04 can be found at [OpenCV Docs](http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html). OpenCV requires [GCC](https://gcc.gnu.org/) and [CMake](http://www.cmake.org/) among other things.

 2. [Ceres solver][] - Install the needed dependencies (download [Google Flags](https://launchpad.net/ubuntu/+source/gflags) and [Google Log](https://launchpad.net/ubuntu/+source/google-glog) and include [SuiteSparse](http://faculty.cse.tamu.edu/davis/suitesparse.html)) and build Ceres according the [documentation](http://ceres-solver.org/building.html). Install Ceres from the ceres-bin directory after `make` by:
 
    sudo make install

 3. [Boost Python][] - Install through apt-get:

    sudo apt-get install libboost-python-dev

 4. [NumPy][], networkx, PyYaml, exifread - Install [pip](https://pypi.python.org/pypi/pip) and then run the following from the root of the project:

    sudo pip install -r requirements.txt

 5. [SciPy][] - Install [gfortran](https://gcc.gnu.org/wiki/GFortran) through apt-get and then install [SciPy][] with:

    sudo apt-get install gfortran
    sudo pip install scipy


## Building inplace using setup.py

    python setup.py build_clib
    python setup.py build_ext --inplace

## Building using CMake

    mkdir build
    cd build
    cmake ../opensfm/src
    make

## Running

An example dataset is available at `data/berlin`.

 1. Put some images in `data/DATASET_NAME/images/`
 2. Put config.yaml in `data/DATASET_NAME/config.yaml`
 3. Go to the root of the project and run `bin/run_all data/DATASET_NAME`
 4. Start an http server from the root with `python -m SimpleHTTPServer`
 5. Browse `http://localhost:8000/viewer/reconstruction.html#file=/data/DATASET_NAME/reconstruction.json`.


[OpenCV]: http://opencv.org/ (Computer vision and machine learning software library)
[NumPy]: http://www.numpy.org/ (Scientific computing with Python)
[SciPy]: http://www.scipy.org/ (Fundamental library for scientific computing)
[Ceres solver]: http://ceres-solver.org/ (Library for solving complicated nonlinear least squares problems)
[boost-python]: http://www.boost.org/
