cmake_minimum_required(VERSION 3.0)

project(opensfm C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif()

set(CMAKE_MODULE_PATH ${opensfm_SOURCE_DIR}/cmake)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} \
    -fPIC \
    -fvisibility=hidden")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
    -fPIC \
    -fvisibility=hidden \
    -fvisibility-inlines-hidden \
    -std=c++11")

# Find dependencies.
add_subdirectory(third_party/pybind11)

include_directories(${PYTHON_INCLUDE_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(LAPACK)
if (LAPACK_FOUND)
  include_directories(${LAPACK_INCLUDE_DIRS})
endif()

find_package(SuiteSparse)
if (SUITESPARSE_FOUND)
  include_directories(${SUITESPARSE_INCLUDE_DIRS})
endif()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})

find_package(Gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIR})

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIR})

find_package(OpenCV)
# OpenCV's OpenCVConfig will enforce imgcodecs for < 3.0 
# (even if OPTIONAL_COMPONENTS) so we remove it as we don't need it
# Cause is imread/imwrite moved to imgcodecs on > 3.0
if(${OpenCV_VERSION} LESS 3.0)
    find_package(OpenCV REQUIRED core imgproc calib3d)
else()
    find_package(OpenCV REQUIRED core imgproc calib3d OPTIONAL_COMPONENTS imgcodecs)
endif()
include_directories(${OpenCV_INCLUDE_DIRS})


# Akaze
include_directories(third_party/akaze/lib/)
set(AKAZE_SRCS
    third_party/akaze/lib/AKAZEConfig.h
    third_party/akaze/lib/AKAZE.h
    third_party/akaze/lib/AKAZE.cpp
    third_party/akaze/lib/fed.h
    third_party/akaze/lib/fed.cpp
    third_party/akaze/lib/nldiffusion_functions.h
    third_party/akaze/lib/nldiffusion_functions.cpp
    third_party/akaze/lib/utils.h
    third_party/akaze/lib/utils.cpp
)
add_library(akaze ${AKAZE_SRCS})
target_link_libraries(akaze ${OpenCV_LIBS})


# VLFeat
include_directories(third_party/vlfeat)
file(GLOB VLFEAT_SRCS third_party/vlfeat/vl/*.c third_party/vlfeat/vl/*.h)
if (NOT CMAKE_SYSTEM_PROCESSOR MATCHES
    "(x86)|(X86)|(x86_64)|(X86_64)|(amd64)|(AMD64)")
    add_definitions(-DVL_DISABLE_SSE2)
endif ()
add_definitions(-DVL_DISABLE_AVX)
add_library(vl ${VLFEAT_SRCS})


# Bundle
include_directories(.)
set(BUNDLE_FILES
    bundle/src/error_utils.h
    bundle/src/motion_prior_terms.h
    bundle/src/absolute_motion_terms.h
    bundle/src/relative_motion_terms.h
    bundle/src/bundle_adjuster.cc)
add_library(bundle ${BUNDLE_FILES})
target_link_libraries(bundle PRIVATE
    ${GFLAGS_LIBRARY}
    ${GLOG_LIBRARY}
    ${CERES_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${SUITESPARSE_LIBRARIES}
)


# Python wrapper
pybind11_add_module(csfm
    csfm.cc
    types.cc
    akaze_bind.cc
    matching.cc
    depthmap.cc
)
target_link_libraries(csfm PRIVATE
    ${OpenCV_LIBS}
    ${GFLAGS_LIBRARY}
    ${GLOG_LIBRARY}
    ${CERES_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${SUITESPARSE_LIBRARIES}
    vl
    akaze
    bundle
)
set_target_properties(csfm PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)


if (CMAKE_BUILD_TYPE MATCHES DEBUG)
    add_executable(debug_c_extension
        debug_c_extension.cc
        third_party/PythonExtensionPatterns/py_import_call_execute.c
    )
    target_link_libraries(debug_c_extension ${PYTHON_LIBRARIES})
endif()


# Tests
option(OPENSFM_BUILD_TESTS "Build OpenSfM unit tests." on)

if (OPENSFM_BUILD_TESTS)
  enable_testing()
  include_directories(third_party/gtest)
  add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

  add_library(gtest
              third_party/gtest/gmock_gtest_all.cc
              third_party/gtest/gmock_main.cc)

  add_library(testing_main testing_main.cc)
  target_link_libraries(testing_main
                        ${GFLAGS_LIBRARY}
                        ${GLOG_LIBRARY}
                        gtest)

  add_executable(depthmap_test depthmap_test.cc depthmap.cc)
  target_link_libraries(depthmap_test
                        ${OpenCV_LIBS}
                        testing_main)
  add_test(depthmap_test depthmap_test)

endif()
