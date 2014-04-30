CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(LABMV C CXX)


SET(LIBMV_PATH ../../../libmv CACHE PATH "Path to libmv source")

INCLUDE_DIRECTORIES(
  .
  ${LIBMV_PATH}/src
  ${LIBMV_PATH}/src/third_party/gtest
  ${LIBMV_PATH}/src/third_party/gtest/include
  ${LIBMV_PATH}/src/third_party/eigen
  ${LIBMV_PATH}/src/third_party/eigen/unsupported
  ${LIBMV_PATH}/src/third_party/glog/src
  ${LIBMV_PATH}/src/third_party/ceres/include
)

LINK_DIRECTORIES(${LIBMV_PATH}/build-g++-release/lib)


ADD_EXECUTABLE(robust_matching robust_matching.cc)
TARGET_LINK_LIBRARIES(robust_matching
                      multiview_test_data
                      multiview
                      numeric
                      pthread
                      gflags
                      glog
                      )

ADD_EXECUTABLE(two_view_reconstruction two_view_reconstruction.cc)
TARGET_LINK_LIBRARIES(two_view_reconstruction
                      multiview_test_data
                      multiview
                      numeric
                      pthread
                      gflags
                      glog
                      )

ADD_EXECUTABLE(bundle bundle.cc)
TARGET_LINK_LIBRARIES(bundle
                      ceres_shared
                      pthread
                      gflags
                      glog
                      jansson
                      )

ADD_EXECUTABLE(bundle_test bundle_test.cc)
TARGET_LINK_LIBRARIES(bundle_test
                      ceres_shared
                      pthread
                      gflags
                      glog
                      jansson
                      gtest
                      )

