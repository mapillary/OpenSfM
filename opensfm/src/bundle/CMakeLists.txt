set(BUNDLE_FILES
    bundle_adjuster.h
    reconstruction_alignment.h
    data/camera.h
    data/data.h
    data/pose.h
    data/rig.h
    data/shot.h
    error/error_utils.h
    error/motion_prior_errors.h
    error/position_functors.h
    error/projection_errors.h
    error/absolute_motion_errors.h
    error/parameters_errors.h
    error/relative_motion_errors.h
    src/bundle_adjuster.cc
)
add_library(bundle ${BUNDLE_FILES})
target_link_libraries(bundle
  PRIVATE
    ${CERES_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${SUITESPARSE_LIBRARIES}
    foundation
)
if (LAPACK_FOUND)
    target_include_directories(bundle PRIVATE ${LAPACK_INCLUDE_DIRS})
endif()
if (SUITESPARSE_FOUND)
    target_include_directories(bundle PRIVATE ${SUITESPARSE_INCLUDE_DIRS})
endif()
target_include_directories(bundle PRIVATE ${CERES_INCLUDE_DIR} ${CMAKE_SOURCE_DIR})

if (OPENSFM_BUILD_TESTS)
    set(BUNDLE_TEST_FILES
        test/reprojection_errors_test.cc
        test/bundle_data_test.cc
    )
    add_executable(bundle_test ${BUNDLE_TEST_FILES})
    target_include_directories(bundle_test PRIVATE ${CMAKE_SOURCE_DIR} ${EIGEN_INCLUDE_DIRS} ${GMOCK_INCLUDE_DIRS})
    target_link_libraries(bundle_test
                        PUBLIC
                        bundle
                        geometry
                        ${TEST_MAIN})
    add_test(bundle_test bundle_test)
endif()

pybind11_add_module(pybundle python/pybind.cc)
target_link_libraries(pybundle PRIVATE
  bundle
  geometry
  foundation
  pybind11)
set_target_properties(pybundle PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)
