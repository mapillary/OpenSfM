cmake_minimum_required(VERSION 2.8)

project(opensfm C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif()

set(CMAKE_MODULE_PATH ${opensfm_SOURCE_DIR}/cmake)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++0x")


# Find dependencies.
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

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost COMPONENTS python REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set(Python_ADDITIONAL_VERSIONS "2.7")
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(NumPy REQUIRED)
include_directories(${NUMPY_INCLUDE_DIRS})


# Libmv
include_directories(third_party)
add_library(mv
    third_party/libmv/multiview/fundamental.cc
    third_party/libmv/multiview/projection.cc
    third_party/libmv/multiview/five_point.cc
    third_party/libmv/multiview/robust_five_point.cc
    third_party/libmv/multiview/triangulation.cc
    third_party/libmv/multiview/conditioning.cc
    third_party/libmv/multiview/panography.cc
    third_party/libmv/multiview/panography_kernel.cc
    third_party/libmv/multiview/robust_panography.cc
    third_party/libmv/numeric/numeric.cc
    third_party/libmv/numeric/poly.cc
)
target_link_libraries(mv
    ${GFLAGS_LIBRARY}
    ${GLOG_LIBRARY}
)


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
add_definitions(-DVL_DISABLE_AVX)
add_library(vl ${VLFEAT_SRCS})


# Python wrapper
add_library(csfm SHARED csfm.cc)
target_link_libraries(csfm
    ${OpenCV_LIBS}
    ${GFLAGS_LIBRARY}
    ${GLOG_LIBRARY}
    ${CERES_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${SUITESPARSE_LIBRARIES}
    vl
    mv
    akaze
)
set_target_properties(csfm PROPERTIES
    PREFIX ""
    SUFFIX ".so"
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)
if(APPLE)
    set_target_properties(csfm PROPERTIES
        LINK_FLAGS "-undefined dynamic_lookup"
    )
    target_link_libraries(csfm "${Boost_LIBRARY_DIR}/libboost_python.a")
else()
    target_link_libraries(csfm
        ${Boost_LIBRARIES}
        ${PYTHON_LIBRARIES}
    )
endif()
