set(SFM_FILES
    observation.h
    tracks_manager.h
    sfm_helpers.h
    types.h
    src/tracks_manager.cc
    src/sfm_helpers.cc
)
add_library(sfm ${SFM_FILES})
target_link_libraries(sfm
  PRIVATE
    foundation
)
target_include_directories(sfm PUBLIC ${EIGEN_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR})

set(SFM_TEST_FILES
    test/tracks_manager_test.cc
)
add_executable(sfm_test ${SFM_TEST_FILES})
target_include_directories(sfm_test PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(sfm_test
                      PUBLIC
                      sfm
                      ${TEST_MAIN})
add_test(sfm_test sfm_test)

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