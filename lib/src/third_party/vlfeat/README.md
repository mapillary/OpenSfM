# VLFeat -- Vision Lab Features Library

> Version 0.9.20

The VLFeat open source library implements popular computer vision
algorithms specialising in image understanding and local featurexs
extraction and matching.  Algorithms incldue Fisher Vector, VLAD,
SIFT, MSER, k-means, hierarchical k-means, agglomerative information
bottleneck, SLIC superpixes, quick shift superpixels, large scale SVM
training, and many others. It is written in C for efficiency and
compatibility, with interfaces in MATLAB for ease of use, and detailed
documentation throughout. It supports Windows, Mac OS X, and Linux.

VLFeat is distributed under the BSD license (see the `COPYING` file).

The documentation is
[available online](http://www.vlfeat.org/index.html) and shipped with
the library as `doc/index.html`. See also:

* [Using with MATLAB](http://www.vlfeat.org/install-matlab.html)
* [Using the command line utilities](http://www.vlfeat.org/install-shell.html)
* [Using the C API](http://www.vlfeat.org/install-c.html)
* [Compiling from source](http://www.vlfeat.org/compiling.html)

## Quick start with MATLAB

To start using VLFeat as a MATLAB toolbox, download the latest VLFeat
[binary package](http://www.vlfeat.org/download/). Note that the
pre-compiled binaries require MATLAB 2009B and later. Unpack it, for
example by using WinZIP (Windows), by double clicking on the archive
(Mac), or by using the command line (Linux and Mac):

    > tar xzf vlfeat-X.Y.Z-bin.tar.gz

Here X.Y.Z denotes the latest version. Start MATLAB and run the
VLFeat setup command:

    > run <VLFEATROOT>/toolbox/vl_setup

Here `<VLFEATROOT>` should be replaced with the path to the VLFeat
directory created by unpacking the archive. All VLFeat demos can now
be run in a row by the command:

    > vl_demo

Check out the individual demos by editing this file: `edit vl_demo`.

## Octave support

The toolbox should be laregly compatible with GNU Octave, an open
source MATLAB equivalent. However, the binary distribution does not
ship with pre-built GNU Octave MEX files. To compile them use

    > cd <vlfeat directory>
    > make MKOCTFILE=<path to the mkoctfile program>

# Changes

- **0.9.20** Maintenance release. Bugfixes.
- **0.9.19** Maintenance release. Minor bugfixes and fixes compilation
  with MATLAB 2014a.
- **0.9.18** Several bugfixes. Improved documentation, particularly of
  the covariant detectors. Minor enhancements of the Fisher vectors.
- **0.9.17** Rewritten SVM implementation, adding support for SGD and
  SDCA optimisers and various loss functions (hinge, squared hinge,
  logistic, etc.) and improving the interface. Added infrastructure to
  support multi-core computations using OpenMP (MATLAB 2009B or later
  required). Added OpenMP support to KD-trees and KMeans. Added new
  Gaussian Mixture Models, VLAD encoding, and Fisher Vector encodings
  (also with OpenMP support). Added LIOP feature descriptors. Added
  new object category recognition example code, supporting several
  standard benchmarks off-the-shelf.
- **0.9.16** Added `VL_COVDET`. This function implements the following
  detectors: DoG, Hessian, Harris Laplace, Hessian Laplace, Multiscale
  Hessian, Multiscale Harris. It also implements affine adaptation,
  estiamtion of feature orientation, computation of descriptors on the
  affine patches (including raw patches), and sourcing of custom
  feature frame.
- **0.9.15** Added `VL_HOG` (HOG features). Added `VL_SVMPEGASOS` and
  a vastly improved SVM implementation. Added `VL_IHASHSUM` (hashed
  counting). Improved INTHIST (integral histogram). Added
  `VL_CUMMAX`. Improved the implementation of `VL_ROC` and
  VL_PR(). Added VL_DET() (Detection Error Trade-off (DET)
  curves). Improved the verbosity control to AIB. Added support for
  Xcode 4.3, improved support for past and future Xcode
  versions. Completed the migration of the old test code in
  `toolbox/test`, moving the functionality to the new unit tests
  `toolbox/xtest`.
- **0.9.14** Added SLIC superpixels. Added VL_ALPHANUM(). Improved
  Windows binary package and added support for Visual
  Studio 2010. Improved the documentation layout and added a proper
  bibliography. Bugfixes and other minor improvements. Moved from the
  GPL to the less restrictive BSD license.
- **0.9.13** Fixed Windows binary package.
- **0.9.12** Fixes `vl_compile` and the architecture string on Linux 32 bit.
- **0.9.11** Fixes a compatibility problem on older Mac OS X versions.
  A few bugfixes are included too.
- **0.9.10** Improves the homogeneous kernel map. Plenty of small
  tweaks and improvements. Make maci64 the default architecture on the
  Mac.
- **0.9.9** Added: sift matching example. Extended Caltech-101
  classification example to use kd-trees.
- **0.9.8** Added: image distance transform, PEGASOS, floating point
  K-means, homogeneous kernel maps, a Caltech-101 classification
  example. Improved documentation.
- **0.9.7** Changed the Mac OS X binary distribution to require a less
  recent version of Mac OS X (10.5).
- **0.9.6** Changed the GNU/Linux binary distribution to require a
  less recent version of the C library.
- **0.9.5** Added kd-tree and new SSE-accelerated vector/histogram
  comparison code.  Improved dense SIFT (dsift) implementation.  Added
  Snow Leopard and MATLAB R2009b support.
- **0.9.4** Added quick shift. Renamed dhog to dsift and improved
  implementation and documentation. Improved tutorials.  Added 64 bit
  Windows binaries. Many other small changes.
- **0.9.3** Namespace change (everything begins with a vl_ prefix
  now). Many other changes to provide compilation support on Windows
  with MATLAB 7.
- **beta-3** Completes to the ikmeans code.
- **beta-2** Many additions.
- **beta-1** Initial public release.
