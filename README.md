OpenSfM
=======

## Overview
OpenSfM is a Structure from Motion library written in Python on top of [OpenCV][]. The library serves as a processing pipeline for reconstructing camera poses and 3D scenes from multiple images. It consists of basic modules for Structure from Motion (feature detection/matching, minimal solvers) with a focus on building a robust and scalable reconstruction pipeline. It also integrates external sensor (e.g. GPS, accelerometer) measurements for geographical alignment and robustness. A JavaScript viewer is provided to preview the models and debug the pipeline.

<p align="center">
  <a href="https://dl.dropboxusercontent.com/u/2801164/public_html/mapillary_blog/navigation/reconstruction.html#file=data/iVioRpbW-oZa0issidL1tg/reconstruction.json.compressed&res=640&img=03PQphaD0hKxVSHwphmobg">
    <img src="https://dl.dropboxusercontent.com/u/2801164/public_html/opensfm/lund.jpg" />
  </a>
</p>

Checkout this [blog post with more demos](http://blog.mapillary.com/update/2014/12/15/sfm-preview.html)


## Dependencies

* [OpenCV][]
* [Ceres Solver][]
* [Boost Python][]
* [NumPy][], [SciPy][], [Networkx][], PyYAML, exifread

### Installing dependencies on MacOSX

Use

    brew tap homebrew/science
    brew install opencv
    brew install homebrew/science/ceres-solver
    brew install boost-python
    sudo pip install -r requirements.txt

Be sure to update your `PYTHONPATH` to include `/usr/local/lib/python2.7/site-packages` where OpenCV has been installed:

    export PYTHONPATH=/usr/local/lib/python2.7/site-packages:$PYTHONPATH


### Installing dependencies on Ubuntu

See the [Dockerfile](https://github.com/mapillary/OpenSfM/blob/master/Dockerfile) for the commands to install all dependencies on Ubuntu 14.04.  The steps are

 1. Install [OpenCV][], [Boost Python][], [NumPy][], [SciPy][] using apt-get
 2. Install python requirements using pip
 3. [Build and Install](http://ceres-solver.org/building.html) the [Ceres solver][] from its source using the `-fPIC` compilation flag


## Building

    python setup.py build


## Running

An example dataset is available at `data/berlin`.

 1. Put some images in `data/DATASET_NAME/images/`
 2. Put config.yaml in `data/DATASET_NAME/config.yaml`
 3. Go to the root of the project and run `bin/run_all data/DATASET_NAME`
 4. Start an http server from the root with `python -m SimpleHTTPServer`
 5. Browse `http://localhost:8000/viewer/reconstruction.html#file=/data/DATASET_NAME/reconstruction.meshed.json`.

Things you can do from there:
- Use datasets with more images
- Click twice on an image to see it. Then use arrows to move between images.
- Run `bin/mesh data/berlin` to build a reconstruction with sparse mesh that will produce smoother transitions from images

# Thanks to sponsors

- Thank you Jetbrains for supporting the project with free licenses for [IntelliJ Ultimate](https://www.jetbrains.com/idea/). Contact peter at mapillary dot com if you are contributor and need one. Apply your own project [here](https://www.jetbrains.com/eforms/openSourceRequest.action?licenseRequest=ideaOSLPRN)

[OpenCV]: http://opencv.org/ (Computer vision and machine learning software library)
[NumPy]: http://www.numpy.org/ (Scientific computing with Python)
[SciPy]: http://www.scipy.org/ (Fundamental library for scientific computing)
[Ceres solver]: http://ceres-solver.org/ (Library for solving complicated nonlinear least squares problems)
[Boost Python]: http://www.boost.org/
[Networkx]: https://github.com/networkx/networkx
