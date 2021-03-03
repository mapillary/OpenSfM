.. Download and install instructions


Building
========

Download
--------

OpenSfM code is available at Github_.  The simplest way to get the code is to clone the repository and its submodules with::

    git clone --recursive https://github.com/mapillary/OpenSfM

If you already have the code or you downloaded a release_, make sure to update the submodules with::

    cd OpenSfM
    git submodule update --init --recursive


Install dependencies
--------------------

OpenSfM depends on the following libraries that need to be installed before building it.

* OpenCV_
* `Ceres Solver`_

Python dependencies can be installed with::

    pip install -r requirements


Installing dependencies on Ubuntu
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

See this `Dockerfile <https://github.com/mapillary/OpenSfM/blob/master/Dockerfile>`_ for the commands to install all dependencies on Ubuntu 20.04.

Installing dependencies on Fedora
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Tested on Fedora 33

    sudo dnf install zlib-devel libjpeg-devel python3-devel g++ ceres-solver-devel opencv-devel eigen3-devel libomp cmake glog-devel

There's an `issue <https://github.com/ceres-solver/ceres-solver/issues/491>`_ with the gflags-config.cmake distributed with Fedora. We need to build from scratch instead of relying on the version installed by dnf:

    mkdir ~/src && cd ~/src && clone git@github.com:gflags/gflags.git && checkout v2.2.2

    mkdir ~/src/gflags/build && cd ~/src/gflags/build && cmake ../ && make -j4

    sudo make install

Install python dependencies before building:

    cd ~/src/OpenSfM && pip install -r requirements.txt

Installing dependencies on MacOSX
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Install OpenCV and the Ceres solver using::

    brew install opencv
    brew install ceres-solver
    brew install libomp
    sudo pip install -r requirements.txt

Make sure you update your ``PYTHONPATH`` to include ``/usr/local/lib/python3.7/site-packages`` where OpenCV have been installed. For example with::

    export PYTHONPATH=/usr/local/lib/python3.7/site-packages:$PYTHONPATH

Also, in order for Cmake to recognize the libraries installed by Brew, make sure that C_INCLUDE_PATH, CPLUS_INCLUDE_PATH, DYLD_LIBRARY_PATH environment variables are set correctly. For example, you can run::

    export C_INCLUDE_PATH=/usr/local/include
    export CPLUS_INCLUDE_PATH=/usr/local/include
    export DYLD_LIBRARY_PATH=$HOME/local/lib64

.. note:: Note on OpenCV 3
    When running OpenSfM on top of OpenCV version 3.0 the `OpenCV Contrib`_ modules are required for extracting SIFT or SURF features.


Building the library
--------------------

Once the dependencies have been installed, you can build OpenSfM by running the following command from the main folder::

    python3 setup.py build


Building the documentation
--------------------------
To build the documentation and browse it locally use::

    python3 setup.py build_doc
    python3 -m http.server --directory build/doc/html/

and browse `http://localhost:8000/ <http://localhost:8000/>`_


.. _Github: https://github.com/mapillary/OpenSfM
.. _release: https://github.com/mapillary/OpenSfM/releases
.. _OpenCV: http://opencv.org/
.. _OpenCV Contrib: https://github.com/itseez/opencv_contrib
.. _NumPy: http://www.numpy.org/
.. _SciPy: http://www.scipy.org/
.. _Ceres solver: http://ceres-solver.org/
.. _Networkx: https://github.com/networkx/networkx
