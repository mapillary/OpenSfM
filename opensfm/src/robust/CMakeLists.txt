set(ROBUST_FILES
    model.h
    random_sampler.h
    robust_estimator.h
    essential_model.h
    line_model.h
    relative_pose_model.h
    src/instanciations.cc
)
add_library(robust ${ROBUST_FILES})
target_link_libraries(robust 
  PRIVATE
    foundation
    geometry
)
target_include_directories(robust PUBLIC ${CMAKE_SOURCE_DIR})

pybind11_add_module(pyrobust python/pybind.cc)
target_link_libraries(pyrobust 
  PRIVATE 
    robust
    foundation
    pybind11
)
set_target_properties(pyrobust PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)