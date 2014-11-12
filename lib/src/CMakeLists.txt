cmake_minimum_required(VERSION 2.8)

project(labmv C CXX)

set(CMAKE_MODULE_PATH ${labmv_SOURCE_DIR}/cmake)

include_directories(.)


find_package(JsonCpp REQUIRED)
include_directories(${JSONCPP_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})

find_package(gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIR})

find_package(glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIR})


add_executable(two_view_reconstruction
               two_view_reconstruction.cc
               libmv/multiview/fundamental.cc
               libmv/multiview/projection.cc
               libmv/multiview/five_point.cc
               libmv/multiview/robust_five_point.cc
               libmv/multiview/triangulation.cc
               libmv/multiview/conditioning.cc
               libmv/numeric/numeric.cc
               libmv/numeric/poly.cc
              )
target_link_libraries(two_view_reconstruction
                      ${GFLAGS_LIBRARY}
                      ${GLOG_LIBRARY}
                      )

add_executable(bundle bundle.cc)
target_link_libraries(bundle
                      ${CERES_LIBRARIES}
                      ${GFLAGS_LIBRARY}
                      ${GLOG_LIBRARY}
                      ${JSONCPP_LIBRARIES}
                      )

#add_executable(bundle_test bundle_test.cc)
#target_link_libraries(bundle_test
#                      ${CERES_LIBRARIES}
#                      ${JSONCPP_LIBRARIES}
#                      ${GTEST_LIBRARY}
#                      )
