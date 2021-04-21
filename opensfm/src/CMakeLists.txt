cmake_minimum_required(VERSION 3.0)

project(opensfm C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif()
set(CMAKE_MODULE_PATH ${opensfm_SOURCE_DIR}/cmake)

####### Compilation Options #######
# Visibility stuff
cmake_policy(SET CMP0063 NEW)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_INLINES ON)

# fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# For compiling VLFeat
add_definitions(-DVL_DISABLE_AVX)

####### Find Dependencies #######
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(LAPACK)
find_package(SuiteSparse)
find_package(Eigen REQUIRED)
find_package(Ceres REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)

find_package(OpenCV)
# OpenCV's OpenCVConfig will enforce imgcodecs for < 3.0
# (even if OPTIONAL_COMPONENTS) so we remove it as we don't need it
# Cause is imread/imwrite moved to imgcodecs on > 3.0
if(${OpenCV_VERSION} LESS 3.0)
    find_package(OpenCV REQUIRED core imgproc calib3d)
else()
    find_package(OpenCV REQUIRED core imgproc calib3d OPTIONAL_COMPONENTS imgcodecs)
endif()

####### Third party libraries #######
add_subdirectory(third_party/pybind11)
add_subdirectory(third_party/akaze)
add_subdirectory(third_party/vlfeat)

####### Debugging #######
if (CMAKE_BUILD_TYPE MATCHES DEBUG)
    add_executable(debug_c_extension
        debug_c_extension.cc
    )
    target_link_libraries(debug_c_extension PRIVATE pybind11::embed)
endif()


####### Testing #######
option(OPENSFM_BUILD_TESTS "Build OpenSfM unit tests." on)

if (OPENSFM_BUILD_TESTS)
  enable_testing()
  include_directories(third_party/gtest)
  add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

  add_library(gtest
              third_party/gtest/gmock_gtest_all.cc
              third_party/gtest/gmock_main.cc)
  target_include_directories(gtest PRIVATE ${GFLAGS_INCLUDE_DIR})

  set(TEST_MAIN test_main)
  add_library(${TEST_MAIN} testing_main.cc)
  target_link_libraries(${TEST_MAIN}
                        ${GFLAGS_LIBRARY}
                        ${GLOG_LIBRARY}
                        gtest)
endif()

####### OpenSfM libraries #######
add_subdirectory(foundation)
add_subdirectory(bundle)
add_subdirectory(dense)
add_subdirectory(features)
add_subdirectory(geo)
add_subdirectory(geometry)
add_subdirectory(robust)
add_subdirectory(sfm)
add_subdirectory(map)
