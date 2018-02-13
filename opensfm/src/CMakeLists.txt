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

option(BUILD_FOR_PYTHON3 "build pyopengv for Python 3" OFF)
set(BOOST_PYTHON3_COMPONENT "python3" CACHE STRING
    "name of the Boost Python 3 component (python3, python-py36 or python-py37)")

if(${BUILD_FOR_PYTHON3})
    set(BOOST_PYTHON_COMPONENT ${BOOST_PYTHON3_COMPONENT})
    set(BOOST_NUMPY_COMPONENT numpy3)
    set(Python_ADDITIONAL_VERSIONS 3.7 3.6)
    set(Python_VERSION_SUFFIX 3)
else()
    set(BOOST_PYTHON_COMPONENT python)
    set(BOOST_NUMPY_COMPONENT numpy)
    set(Python_ADDITIONAL_VERSIONS 2.7)
    set(Python_VERSION_SUFFIX "")
endif()

find_package(Boost REQUIRED COMPONENTS ${BOOST_PYTHON_COMPONENT})

if(${Boost_VERSION} LESS 106300)
    message("Not using boost/python/numpy.")
    set(USE_BOOST_PYTHON_NUMPY "off")
else()
    message("Using boost/python/numpy.")
    set(USE_BOOST_PYTHON_NUMPY "on")
    add_definitions(-DUSE_BOOST_PYTHON_NUMPY)

    find_package(Boost 1.63 REQUIRED COMPONENTS
                 ${BOOST_PYTHON_COMPONENT} ${BOOST_NUMPY_COMPONENT})
endif()

find_package(PythonLibs ${Python_VERSION_SUFFIX} REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(NumPy REQUIRED)
include_directories(${NUMPY_INCLUDE_DIRS})


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


# Python wrapper
add_library(csfm SHARED csfm.cc)
target_link_libraries(csfm
    ${OpenCV_LIBS}
    ${GFLAGS_LIBRARY}
    ${GLOG_LIBRARY}
    ${CERES_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${SUITESPARSE_LIBRARIES}
    ${Boost_LIBRARIES}
    vl
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
else()
    target_link_libraries(csfm
        ${PYTHON_LIBRARIES}
    )
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

  macro (OPENSFM_TEST NAME EXTRA_LIBS)
    add_executable(${NAME}_test ${NAME}_test.cc)
    target_link_libraries(${NAME}_test
                          ${EXTRA_LIBS} # Extra libs MUST be first.
                          ${GFLAGS_LIBRARY}
                          ${GLOG_LIBRARY}
                          gtest
                          testing_main)
    add_test(${NAME}_test ${NAME}_test)
  endmacro ()

  opensfm_test(depthmap "${OpenCV_LIBS}")
endif()
