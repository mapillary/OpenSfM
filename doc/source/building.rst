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
* OpenMP

Python dependencies can be installed with::

    pip install -r requirements.txt

Still, we recommand using a full virtual environment manager such as Anaconda, not to mess up with your current setup. Anaconda will take care of installing both systems and python dependencies.
Anaconda dependencies installation has been tested under MacOS (Sequoia) and Ubuntu 24.04, and can be installed with::

    conda env create --file conda.yml --yes

Installing dependencies on Ubuntu
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

See this `Dockerfile <https://github.com/mapillary/OpenSfM/blob/main/Dockerfile>`_ for the commands to install all dependencies on Ubuntu 20.04.

Installing dependencies on Fedora
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Install conda using dnf with::

    sudo dnf install conda
    conda init

Close and re-open your terminal

Install the conda environment with:

    conda env create --file conda.yml --yes

Activate the conda environment with::

    conda activate opensfm

Installing dependencies on MacOSX
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Install conda using brew with::

    brew install --cask anaconda
    export PATH='/Users/USERNAME/anaconda3/bin:$PATH'
    conda init

Close and re-open your terminal

Install the conda environment with:

    conda env create --file conda.yml --yes

Activate the conda environment with::

    conda activate opensfm

.. note:: Note on OpenCV 3
    When running OpenSfM on top of OpenCV version 3.0 the `OpenCV Contrib`_ modules are required for extracting SIFT or SURF features.


Installing dependencies on Windows
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Install git_.

Install cmake_.

Install `Visual Studio 2019`_ from "Older Downloads" (CMake does not currently support VS 2022). Make sure to select the "Desktop development with C++" option.

Install `Python 3.8`_ from the Microsoft Store (OpenSFM uses classes that have been removed in Python 3.9).

Install vcpkg from the OpenSfM root directory::

    cd OpenSfM
    git clone https://github.com/microsoft/vcpkg
    cd vcpkg
    bootstrap-vcpkg.bat

Then install OpenCV, Ceres, SuiteSparse and LAPACK (this will take a while)::

    vcpkg install opencv4 ceres ceres[suitesparse] lapack suitesparse --triplet x64-windows

Finally install the PIP requirements::

    pip install -r requirements.txt


Building the library
--------------------

Once the dependencies have been installed, you can build OpenSfM by running the following command from the main folder::

    python setup.py build

Building Docker images
----------------------

Once dependencies have been installed, you can build OpenSfM Docker images by running the following command from the main folder::

    docker build -t opensfm -f Dockerfile .

To build an image using the Ceres 2 solver, use::

  docker build -t opensfm:ceres2 -f Dockerfile.ceres2 .

Building the documentation
--------------------------
To build the documentation and browse it locally use::

    pip install sphinx_rtd_theme
    python setup.py build_doc
    python -m http.server --directory build/doc/html/

and browse `http://localhost:8000/ <http://localhost:8000/>`_


.. _Github: https://github.com/mapillary/OpenSfM
.. _release: https://github.com/mapillary/OpenSfM/releases
.. _OpenCV: http://opencv.org/
.. _OpenCV Contrib: https://github.com/itseez/opencv_contrib
.. _NumPy: http://www.numpy.org/
.. _SciPy: http://www.scipy.org/
.. _Ceres solver: http://ceres-solver.org/
.. _Networkx: https://github.com/networkx/networkx
.. _git: https://git-scm.com/
.. _cmake: https://cmake.org/
.. _Visual Studio 2019: https://visualstudio.microsoft.com/downloads/
.. _Python 3.8: https://www.microsoft.com/en-us/p/python-38/9mssztt1n39l
