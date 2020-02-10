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

if (NOT CMAKE_SYSTEM_PROCESSOR MATCHES
    "(x86)|(X86)|(x86_64)|(X86_64)|(amd64)|(AMD64)")
    add_definitions(-DVL_DISABLE_SSE2)
endif ()
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

# Akaze
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
target_include_directories(third_party/akaze/lib/)
target_include_directories(${OpenCV_INCLUDE_DIRS})


# VLFeat
file(GLOB VLFEAT_SRCS third_party/vlfeat/vl/*.c third_party/vlfeat/vl/*.h)
target_include_directories(vl third_party/vlfeat)
add_library(vl ${VLFEAT_SRCS})

####### OpenSfM libraries #######

# Bundle
include_directories(bundle/src)
set(BUNDLE_FILES
    bundle/bundle_adjuster.h
    bundle/reconstruction_alignment.h
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
if (LAPACK_FOUND)
    target_include_directories(bundle ${LAPACK_INCLUDE_DIRS})
endif()
if (SUITESPARSE_FOUND)
    target_include_directories(bundle ${SUITESPARSE_INCLUDE_DIRS})
endif()
target_include_directories(bundle ${EIGEN_INCLUDE_DIRS} ${CERES_INCLUDE_DIR} ${GFLAGS_INCLUDE_DIR} ${GLOG_INCLUDE_DIR})

pybind11_add_module(pybundle bundle/python/pybind.cc)
target_link_libraries(pybundle PRIVATE bundle)
set_target_properties(pybundle PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)

# Dense
set(DENSE_FILES
    dense/depthmap_bind.h
    dense/depthmap.h
    dense/src/depthmap_test.cc
    dense/src/depthmap_test.cc
add_library(dense ${DENSE_FILES})
target_link_libraries(dense PRIVATE
    ${OpenCV_LIBS}
    ${GFLAGS_LIBRARY}
    ${GLOG_LIBRARY}
)
target_include_directories(dense ${EIGEN_INCLUDE_DIRS} ${OpenCV_LIBS} ${GFLAGS_INCLUDE_DIR} ${GLOG_INCLUDE_DIR})
pybind11_add_module(pydense dense/python/pybind.cc)
target_link_libraries(pydense PRIVATE dense)
target_include_directories(pydense ${PYTHON_INCLUDE_DIRS})
set_target_properties(pydense PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)

# Features
set(FEATURES_FILES
    features/akaze_bind.h
    features/hahog.h
    features/matching.h
    features/src/akaze_bind.cc
    features/src/hahog.cc
    features/src/matching.cc
add_library(features ${FEATURES_FILES})
target_link_libraries(features PRIVATE
    ${OpenCV_LIBS}
    ${GFLAGS_LIBRARY}
    ${GLOG_LIBRARY}
    akaze
    vl
)
target_include_directories(features ${EIGEN_INCLUDE_DIRS} ${OpenCV_LIBS} ${GFLAGS_INCLUDE_DIR} ${GLOG_INCLUDE_DIR})
pybind11_add_module(pyfeatures features/python/pybind.cc)
target_link_libraries(pyfeatures PRIVATE features)
target_include_directories(pyfeatures ${PYTHON_INCLUDE_DIRS})
set_target_properties(pyfeatures PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)

# Geometry
set(GEOMETRY_FILES
    geometry/essential.h
    geometry/pose.h
    geometry/triangulation.h
    geometry/src/triangulation.cc
add_library(geometry ${FEATURES_FILES})
target_include_directories(features ${EIGEN_INCLUDE_DIRS})
pybind11_add_module(pygeometry geometry/python/pybind.cc)
target_link_libraries(pygeometry PRIVATE geometry)
target_include_directories(pygeometry ${PYTHON_INCLUDE_DIRS})
set_target_properties(pygeometry PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)

# Robust
set(ROBUST_FILES
    robust/model.h
    robust/random_sampler.h
    robust/robust_estimator.h
    robust/essential_model.h
    robust/line_model.h
    robust/relative_pose_model.h
    robust/src/scorer.cc
    robust/src/instanciations.cc
add_library(robust ${ROBUST_FILES})
target_include_directories(features ${EIGEN_INCLUDE_DIRS})
pybind11_add_module(pyrobust robust/python/pybind.cc)
target_link_libraries(pyrobust PRIVATE robust)
target_include_directories(pyrobust ${PYTHON_INCLUDE_DIRS})
set_target_properties(pyrobust PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)

####### Debugging #######
if (CMAKE_BUILD_TYPE MATCHES DEBUG)
    add_executable(debug_c_extension
        debug_c_extension.cc
        third_party/PythonExtensionPatterns/py_import_call_execute.c
    )
    target_link_libraries(debug_c_extension ${PYTHON_LIBRARIES})
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

  add_library(testing_main testing_main.cc)
  target_link_libraries(testing_main
                        ${GFLAGS_LIBRARY}
                        ${GLOG_LIBRARY}
                        gtest)

  add_executable(depthmap_test dense/src/depthmap_test.cc dense/src/depthmap.cc)
  target_link_libraries(depthmap_test
                        ${OpenCV_LIBS}
                        testing_main)
  add_test(depthmap_test depthmap_test)

endif()
