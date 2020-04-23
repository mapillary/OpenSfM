# Changelog


## [Unreleased]


## 0.4.0

### Added
 - Internal geometric solvers and RANSAC

### Changed
- Undistorted images, masks and depthmaps are now under the undistorted folder.  Use the `bin/migrate_undistort.sh` script to port old datasets to the new folder structure.
- Removed dependency on opengv
- Restructured c++ code

### Improved
- Undistorting 16bit images now produces 16bit undistorted images. @pierotofy
- Fix bug on multiple reconstruction alignment. @linusmartensson


## 0.3.0

### Added
- Matching:
  - Bag-of-words pair selection
  - VLAD pair selection
  - Words based matching

### Changed
- Local bundle adjustment is used by default
- extract_metadata extracts exif only once
- Changed default FLANN parameters for faster, less precise matching
- Require cmake version 3 or later 

### Improved
- Faster local bundle adjustment by limiting the neighborhood and the number of iterations
- New GCP file format allows for GCPs without known world coordinates


## 0.2.0

### Added
- Python 3 support. @spease
- Brown-Conrady camera model. @bdholt1 
- reporting
- docs on coordinates systems and incremental reconstruction pipeline. @jdjakub, @bryanibit 
- export reconstruction in proj4 geographic coordinates

### Improved
- improved initialization for planar scenes
- support for recent (1.65+) versions of boost python. @spease
- fix bug when using GCP. @jhacsonmeza 
- better navigation for 360 images. @mpetroff
- use loky for better exception handling when using multi-processing. @pierotofy 


## 0.1.0

### Added
- Add option `local_bundle_radius` to run bundle adjustment locally around the last added image
- Add option`optimize_camera_parameters` to optimize or not the internal camera parameters
- Add tools to split large datasets into multiple submodels and align them a posteriori

### Changed
- `run_all` command is now named `opensfm_run_all`
- Features are now saved in the `features` folder. The feature type is no longer used to name the feature folder and files
- The option `bundle_use_gps` is now `true` by default.
- Update openMVS exporter with current master `Interface.h`

### Improved
- Speed up the search for candidate matching images
- Open binary file in binary mode under Windows


## 0.0.0

Initial release.