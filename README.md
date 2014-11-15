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
    brew install jhead
    sudo pip install -r requirements.txt



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
