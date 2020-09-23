set(DENSE_FILES
    depthmap_bind.h
    depthmap.h
    src/depthmap.cc
)
add_library(dense ${DENSE_FILES})
target_link_libraries(dense PRIVATE foundation)
target_include_directories(dense PRIVATE ${CMAKE_SOURCE_DIR})

if (OPENSFM_BUILD_TESTS)
    set(DENSE_TEST_FILES
        test/depthmap_test.cc
    )
    add_executable(dense_test ${DENSE_TEST_FILES})
    target_include_directories(dense_test PRIVATE ${CMAKE_SOURCE_DIR} ${GTEST_INCLUDE_DIRS})
    target_link_libraries(dense_test
                        PUBLIC
                        dense
                        ${TEST_MAIN}
                        ${OpenCV_LIBS})
    add_test(dense_test dense_test)
endif()

pybind11_add_module(pydense python/pybind.cc)
target_include_directories(pydense PRIVATE ${GLOG_INCLUDE_DIR})
target_link_libraries(pydense PRIVATE dense foundation pybind11)
set_target_properties(pydense PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)
