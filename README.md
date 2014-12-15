OpenSfM
=======

Structure from Motion library written in Python on top of OpenCV.


Dependencies
------------
* [OpenCV][]
* [Ceres Solver][]
* [JsonCpp][]
* [SciPy][]
* [NumPy][], networkx, PyYAML, exifread

### Dependencies included in source

 * Parts of [libmv](https://github.com/libmv/libmv).

### Installing dependencies on MacOSX

Use

    brew tap homebrew/science
    brew info opencv
    
It will show information about opencv package. To install, usually `brew install opencv` is sufficient, but to enable import from python modules, you should read 'Caveats' section saying:

        ==> Caveats
        Python modules have been installed and Homebrew's site-packages is not
        in your Python sys.path, so you will not be able to import the modules
        this formula installed. If you plan to develop with these modules,
        please run:
          mkdir -p /Users/someuser/Library/Python/2.7/lib/python/site-packages
          echo 'import site; site.addsitedir("/usr/local/lib/python2.7/site-packages")' >> /Users/someuser/Library/Python/2.7/lib/python/site-packages/homebrew.pth

Run these commands and OpenCV will be added to python site packages load path.

Then:

    brew install homebrew/science/ceres-solver
    brew tap cuber/homebrew-jsoncpp
    brew install jsoncpp
    sudo pip install -r requirements.txt

### Installing dependencies on Ubuntu

 1. [OpenCV][] - Install by following the steps in the Ubuntu OpenCV  [installation guide](https://help.ubuntu.com/community/OpenCV). An alternative instruction tested for Ubuntu 10.04 can be found at [OpenCV Docs](http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html). OpenCV requires [GCC](https://gcc.gnu.org/) and [CMake](http://www.cmake.org/) among other things.

 2. [Ceres solver][] - Install the needed dependencies (download [Google Flags](https://launchpad.net/ubuntu/+source/gflags) and [Google Log](https://launchpad.net/ubuntu/+source/google-glog) and include [SuiteSparse](http://faculty.cse.tamu.edu/davis/suitesparse.html)) and build Ceres according the [documentation](http://ceres-solver.org/building.html). Install Ceres from the ceres-bin directory after `make` by:
 
        sudo make install

 3. [JsonCpp][] - Install through apt-get:

        sudo apt-get install libjsoncpp-dev

 4. [NumPy][], networkx, PyYaml, exifread - Install [pip](https://pypi.python.org/pypi/pip) and then run the following from the root of the project:

        sudo pip install -r requirements.txt

 5. [SciPy][] - Install [gfortran](https://gcc.gnu.org/wiki/GFortran) through apt-get and then install [SciPy][] with:

        sudo apt-get install gfortran
        sudo pip install scipy

Building
--------
1. `cd lib; mkdir build; cd build`
2. `cmake ../src`
3. `make`


Running
-------
An example dataset is available at data/berlin.

 1. Put some images in `data/DATASET_NAME/images/`
 2. Put config.yaml at `data/DATASET_NAME/config.yaml`
 3. Go to the root of the project and run `bin/run_all data/DATASET_NAME`
 4. Start an http server from the root with `python -m SimpleHTTPServer`
 5. Browse `http://localhost:8000/viewer/reconstruction.html#file=/data/DATASET_NAME/reconstruction.json`.



[OpenCV]: http://opencv.org/ (Computer vision and machine learning software library)
[NumPy]: http://www.numpy.org/ (Scientific computing with Python)
[SciPy]: http://www.scipy.org/ (Fundamental library for scientific computing)
[Ceres solver]: http://ceres-solver.org/ (Library for solving complicated nonlinear least squares problems)
[JsonCpp]: https://github.com/open-source-parsers/jsoncpp (C++ library that allows manipulating JSON values)
