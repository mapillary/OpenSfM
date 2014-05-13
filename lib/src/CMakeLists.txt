cmake_minimum_required(VERSION 2.8)

project(labmv C CXX)

set(CMAKE_MODULE_PATH ${labmv_SOURCE_DIR}/cmake)

set(LIBMV_PATH ../../../libmv CACHE PATH "Path to libmv source")

include_directories(
  .
  ${LIBMV_PATH}/src
  ${LIBMV_PATH}/src/third_party/gtest
  ${LIBMV_PATH}/src/third_party/gtest/include
  ${LIBMV_PATH}/src/third_party/eigen
  ${LIBMV_PATH}/src/third_party/eigen/unsupported
  ${LIBMV_PATH}/src/third_party/glog/src
  ${LIBMV_PATH}/src/third_party/ceres/include
)

link_directories(${LIBMV_PATH}/bin-opt/lib)

find_package(JsonCpp)
include_directories(${JSONCPP_INCLUDE_DIRS})

add_executable(robust_matching robust_matching.cc)
target_link_libraries(robust_matching
                      multiview
                      numeric
                      pthread
                      gflags
                      glog
                      )

add_executable(two_view_reconstruction two_view_reconstruction.cc)
target_link_libraries(two_view_reconstruction
                      multiview
                      numeric
                      pthread
                      gflags
                      glog
                      )

add_executable(bundle bundle.cc)
target_link_libraries(bundle
                      ceres_shared
                      pthread
                      gflags
                      glog
                      ${JSONCPP_LIBRARIES}
                      )

add_executable(bundle_test bundle_test.cc)
target_link_libraries(bundle_test
                      ceres_shared
                      pthread
                      gflags
                      glog
                      ${JSONCPP_LIBRARIES}
                      gtest
                      )
