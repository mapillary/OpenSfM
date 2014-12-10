cmake_minimum_required(VERSION 2.8)

project(opensfm C CXX)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE)
endif()

set(CMAKE_MODULE_PATH ${opensfm_SOURCE_DIR}/cmake)

include_directories(. third_party)

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

find_package(JsonCpp REQUIRED)
include_directories(${JSONCPP_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})

find_package(Gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIR})

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

add_executable(two_view_reconstruction
               two_view_reconstruction.cc
               third_party/libmv/multiview/fundamental.cc
               third_party/libmv/multiview/projection.cc
               third_party/libmv/multiview/five_point.cc
               third_party/libmv/multiview/robust_five_point.cc
               third_party/libmv/multiview/triangulation.cc
               third_party/libmv/multiview/conditioning.cc
               third_party/libmv/numeric/numeric.cc
               third_party/libmv/numeric/poly.cc
              )
target_link_libraries(two_view_reconstruction
                      ${GFLAGS_LIBRARY}
                      ${GLOG_LIBRARY}
                      )

add_executable(bundle bundle.cc bundle.h)
target_link_libraries(bundle
                      ${CERES_LIBRARIES}
                      ${GFLAGS_LIBRARY}
                      ${GLOG_LIBRARY}
                      ${JSONCPP_LIBRARIES}
                      ${LAPACK_LIBRARIES}
                      ${SUITESPARSE_LIBRARIES}
                      )

#add_executable(bundle_test bundle_test.cc)
#target_link_libraries(bundle_test
#                      ${CERES_LIBRARIES}
#                      ${JSONCPP_LIBRARIES}
#                      ${GTEST_LIBRARY}
#                      )


# AKAZE libraries
include_directories("third_party/akaze/lib/")

set(AKAZE_SRCS
    third_party/akaze/lib/AKAZEConfig.h
    third_party/akaze/lib/AKAZE.h
    third_party/akaze/lib/AKAZE.cpp
    third_party/akaze/lib/fed.h
    third_party/akaze/lib/fed.cpp
    third_party/akaze/lib/nldiffusion_functions.h
    third_party/akaze/lib/nldiffusion_functions.cpp
    third_party/akaze/lib/utils.h
    third_party/akaze/lib/utils.cpp)

add_library(AKAZE_LIB ${AKAZE_SRCS})
target_link_libraries(AKAZE_LIB ${OpenCV_LIBS})

add_executable(akaze
               akaze.cpp)
target_link_libraries(akaze AKAZE_LIB)
