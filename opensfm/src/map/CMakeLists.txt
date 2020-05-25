set(MAP_FILES
  shot.h
  landmark.h
  map.h
  defines.h
  src/landmark.cc
  src/map.cc
  src/shot.cc
  #map_io.h
  #src/map_io.cc
)
add_library(map ${MAP_FILES})
target_link_libraries(map
  PUBLIC
    # pybind11
    ${OpenCV_LIBS}
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
    # nlohmann_json::nlohmann_json
    # pybind11
    # ${OpenCV_LIBS}
)
set_target_properties(pymap PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)