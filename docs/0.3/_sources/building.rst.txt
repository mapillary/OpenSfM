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
* OpenGV_
* `Ceres Solver`_
* NumPy_, SciPy_, Networkx_, PyYAML, exifread


Installing dependencies on Ubuntu
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If using Python 3, see this `Dockerfile <https://github.com/paulinus/opensfm-docker-base/blob/master/Dockerfile.python2>`_ for the commands to install all dependencies on Ubuntu 18.04.

If using Python 2, follow the process here `Dockerfile.python2 <https://github.com/paulinus/opensfm-docker-base/blob/master/Dockerfile.python2>`_.

The main steps are

1. Install OpenCV, NumPy, SciPy using apt-get
2. Install python requirements using pip
3. Clone, build and install OpenGV following the receipt in the Dockerfile
4. `Build and Install <http://ceres-solver.org/installation.html>`_ the Ceres solver from its source using the ``-fPIC`` compilation flag.


Installing dependencies on MacOSX
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Install OpenCV and the Ceres solver using::

    brew install opencv
    brew install ceres-solver
    sudo pip install -r requirements.txt

And install OpenGV using::

    brew install eigen
    git clone --recurse-submodules -j8 https://github.com/paulinus/opengv.git
    cd opengv/build
    cmake .. -DBUILD_TESTS=OFF -DBUILD_PYTHON=ON
    make install

Make sure you update your ``PYTHONPATH`` to include ``/usr/local/lib/python2.7/site-packages`` where OpenCV and OpenGV have been installed. For example with::

    export PYTHONPATH=/usr/local/lib/python2.7/site-packages:$PYTHONPATH


.. note:: Note on OpenCV 3
    When running OpenSfM on top of OpenCV version 3.0 the `OpenCV Contrib`_ modules are required for extracting SIFT or SURF features.


Building the library
--------------------

Once the dependencies have been installed, you can build OpenSfM by running the following command from the main folder::

    python setup.py build

or ``python3 setup.py build`` for a Python 3 build.

Building the documentation
--------------------------
To build the documentation and browse it locally use::

    cd doc
    make livehtml

and browse `http://localhost:8001/ <http://localhost:8001/>`_


.. _Github: https://github.com/mapillary/OpenSfM
.. _release: https://github.com/mapillary/OpenSfM/releases
.. _OpenCV: http://opencv.org/
.. _OpenCV Contrib: https://github.com/itseez/opencv_contrib
.. _OpenGV: http://laurentkneip.github.io/opengv/
.. _NumPy: http://www.numpy.org/
.. _SciPy: http://www.scipy.org/
.. _Ceres solver: http://ceres-solver.org/
.. _Networkx: https://github.com/networkx/networkx


