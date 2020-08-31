set(ROBUST_FILES
    model.h
    random_sampler.h
    robust_estimator.h
    essential_model.h
    line_model.h
    absolute_pose_model.h
    relative_pose_model.h
    relative_rotation_model.h
    absolute_pose_known_rotation_model.h
    src/absolute_pose_model.cc
    src/absolute_pose_known_rotation_model.cc
    src/relative_rotation_model.cc
    src/relative_pose_model.cc
    src/line_model.cc
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
target_include_directories(pyrobust PRIVATE ${GLOG_INCLUDE_DIR})
target_link_libraries(pyrobust
  PRIVATE
    robust
    foundation
    pybind11
)
set_target_properties(pyrobust PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)
