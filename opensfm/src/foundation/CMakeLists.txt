set(FOUNDATION_FILES
    stl_extensions.h
    types.h
    newton_raphson.h
    numeric.h
    optional.h
    union_find.h
    src/types.cc
    src/newton_raphson.cc
    src/numeric.cc
)
add_library(foundation ${FOUNDATION_FILES})
target_link_libraries(foundation
  PUBLIC
    pybind11
    ${OpenCV_LIBS}
    ${OpenMP_libomp_LIBRARY}
  PRIVATE
    ${GFLAGS_LIBRARY}
    ${GLOG_LIBRARY}
)
target_include_directories(foundation
  PUBLIC
    ${EIGEN_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
    ${OpenMP_CXX_INCLUDE_DIR}
)

if (OPENSFM_BUILD_TESTS)
    set(FOUNDATION_TEST_FILES
        test/newton_raphson_test.cc
        test/union_find_test.cc
    )
    add_executable(foundation_test ${FOUNDATION_TEST_FILES})
    target_include_directories(foundation_test PRIVATE ${CMAKE_SOURCE_DIR} ${GMOCK_INCLUDE_DIRS})
    target_link_libraries(foundation_test
                          PUBLIC
                          foundation
                          ${TEST_MAIN})
    add_test(foundation_test foundation_test)
endif()
