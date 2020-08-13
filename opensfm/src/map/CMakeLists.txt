set(MAP_FILES
  shot.h
  landmark.h
  map.h
  ground_control_points.h
  defines.h
  src/landmark.cc
  src/map.cc
  src/shot.cc
  dataviews.h
  src/dataviews.cc
  ba_helpers.h
  src/ba_helpers.cc
)

add_library(map ${MAP_FILES})
target_link_libraries(map
  PUBLIC
    geometry
)

target_include_directories(map 
  PUBLIC
    ${EIGEN_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
)

pybind11_add_module(pymap python/pybind.cc)
target_link_libraries(pymap 
  PRIVATE
    map
    geometry
    bundle
)

set(MAP_TEST_FILES
  test/map_test.cc
)

add_executable(map_test ${MAP_TEST_FILES})
target_include_directories(map_test PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(map_test
                      PUBLIC
                      geometry
                      map
                      ${TEST_MAIN})
add_test(map_test map_test)

set_target_properties(pymap PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)