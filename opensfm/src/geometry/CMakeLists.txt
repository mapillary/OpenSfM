# Geometry
set(GEOMETRY_FILES
    essential.h
    pose.h
    triangulation.h
    src/essential.cc
    src/triangulation.cc
    src/pose.cc
)
add_library(geometry ${GEOMETRY_FILES})
target_link_libraries(geometry 
  PRIVATE 
    foundation
)
target_include_directories(geometry PUBLIC ${CMAKE_SOURCE_DIR})

pybind11_add_module(pygeometry python/pybind.cc)
target_link_libraries(pygeometry 
  PRIVATE
    geometry
    foundation
    pybind11
)
set_target_properties(pygeometry PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)