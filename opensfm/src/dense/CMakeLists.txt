set(DENSE_FILES
    depthmap_bind.h
    depthmap.h
    src/depthmap.cc
)
add_library(dense ${DENSE_FILES})
target_link_libraries(dense PRIVATE foundation)
target_include_directories(dense PRIVATE ${CMAKE_SOURCE_DIR})

pybind11_add_module(pydense python/pybind.cc)
target_link_libraries(pydense PRIVATE dense foundation pybind11)
set_target_properties(pydense PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)