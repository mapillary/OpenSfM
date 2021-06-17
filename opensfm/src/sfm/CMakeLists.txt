set(SFM_FILES
    retriangulation.h
    ba_helpers.h
    tracks_helpers.h
    src/retriangulation.cc
    src/ba_helpers.cc
    src/tracks_helpers.cc
)
add_library(sfm ${SFM_FILES})
target_link_libraries(sfm
  PRIVATE
    foundation
    map
    bundle
)
target_include_directories(sfm PUBLIC ${EIGEN_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR})

if (OPENSFM_BUILD_TESTS)
    set(SFM_TEST_FILES
        test/tracks_helpers_test.cc
    )
    add_executable(sfm_test ${SFM_TEST_FILES})
    target_include_directories(sfm_test PRIVATE ${CMAKE_SOURCE_DIR})
    target_link_libraries(sfm_test
                        PUBLIC
                        sfm
                        ${TEST_MAIN})
    add_test(sfm_test sfm_test)
endif()

pybind11_add_module(pysfm python/pybind.cc)
target_include_directories(pysfm PRIVATE ${GLOG_INCLUDE_DIR})
target_link_libraries(pysfm
  PRIVATE
    foundation
    pybind11
    sfm
)
set_target_properties(pysfm PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)
