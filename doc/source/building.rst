.. Notes and doc on dense matching


Building
========

OpenSfM code is available at Github_.

OpenSfM depends on the following libraries that need to be installed before building it.

* OpenCV_
* OpenGV_
* `Ceres Solver`_
* `Boost Python`_
* NumPy_, SciPy_, Networkx_, PyYAML, exifread

Once the dependencies have been installed, you can build OpenSfM by running the following command from the main folder::

    python setup.py build


Installing dependencies on Ubuntu
---------------------------------

See this `Dockerfile <https://github.com/paulinus/opensfm-docker-base/blob/master/Dockerfile>`_ for the commands to install all dependencies on Ubuntu 14.04.  The steps are

1. Install OpenCV, Boost Python, NumPy, SciPy using apt-get
2. Install python requirements using pip
3. Clone, build and install OpenGV following the receipt in the Dockerfile
4. `Build and Install <http://ceres-solver.org/installation.html>`_ the Ceres solver from its source using the ``-fPIC`` compilation flag.


Installing dependencies on MacOSX
---------------------------------

Install OpenCV, boost python and the Ceres solver using::

    brew tap homebrew/science
    brew install opencv
    brew install homebrew/science/ceres-solver
    brew install boost-python
    sudo pip install -r requirements.txt

And install OpenGV using::

    brew install eigen
    git clone https://github.com/paulinus/opengv.git
    cd opengv/build
    cd opengv/build
    cmake .. -DBUILD_TESTS=OFF -DBUILD_PYTHON=ON
    make install

Be sure to update your ``PYTHONPATH`` to include ``/usr/local/lib/python2.7/site-packages`` where OpenCV and OpenGV have been installed. For example::

    export PYTHONPATH=/usr/local/lib/python2.7/site-packages:$PYTHONPATH


Note on OpenCV 3
----------------

When running OpenSfM on top of OpenCV version 3.0 the `OpenCV Contrib`_ modules are required for extracting SIFT or SURF features.


.. _Github: https://github.com/mapillary/OpenSfM
.. _OpenCV: http://opencv.org/
.. _OpenCV Contrib: https://github.com/itseez/opencv_contrib
.. _OpenGV: http://laurentkneip.github.io/opengv/
.. _NumPy: http://www.numpy.org/
.. _SciPy: http://www.scipy.org/
.. _Ceres solver: http://ceres-solver.org/
.. _Boost Python: http://www.boost.org/
.. _Networkx: https://github.com/networkx/networkx


Building the documentation
--------------------------
To build the documentation and browse it locally use::

    cd doc
    make livehtml

and browse `http://localhost:8001/ <http://localhost:8001/>`_
