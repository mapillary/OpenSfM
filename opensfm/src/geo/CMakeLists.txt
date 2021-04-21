# Geo
set(GEO_FILES
    geo.h
    src/geo.cc
)
add_library(geo ${GEO_FILES})
target_link_libraries(geo
  PRIVATE
    foundation
)
target_include_directories(geo PUBLIC ${CMAKE_SOURCE_DIR})

if (OPENSFM_BUILD_TESTS)
    set(GEO_TEST_FILES
        test/geo_test.cc
    )
    add_executable(geo_test ${GEO_TEST_FILES})
    target_include_directories(geo_test PRIVATE ${CMAKE_SOURCE_DIR} ${EIGEN_INCLUDE_DIRS})
    target_link_libraries(geo_test
                        PUBLIC
                        geo
                        ${TEST_MAIN})
    add_test(geo_test geo_test)
endif()

pybind11_add_module(pygeo python/pybind.cc)
target_link_libraries(pygeo
  PRIVATE
    geo
    foundation
    pybind11
)
set_target_properties(pygeo PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)
