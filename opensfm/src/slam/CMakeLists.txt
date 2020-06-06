set(ORB_FILES
  orb_extractor_bind.h
  third_party/orb_extractor/orb_extractor.cc
  third_party/orb_extractor/orb_extractor.h
  third_party/orb_extractor/orb_extractor_node.cc
  third_party/orb_extractor/orb_extractor_node.h
  third_party/orb_extractor/orb_params.cc
  third_party/orb_extractor/orb_params.h
  third_party/orb_extractor/orb_point_pairs.h
)
set(SLAM_FILES
  slam_utilities.h
  src/slam_utilities.cc
  guided_matching_bind.h
  guided_matching.h
  src/guided_matching.cc
  slam_map.h
  src/slam_map.cc
  pyslam_utilities.h
)
add_library(slam ${SLAM_FILES} ${ORB_FILES})
target_link_libraries(slam
  PUBLIC
    map
    geometry
    foundation
    sfm
    ${OpenCV_LIBS}
)

target_include_directories(slam 
  PUBLIC
    ${EIGEN_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
)
pybind11_add_module(pyslam python/pybind.cc)
target_link_libraries(pyslam 
  PRIVATE
    map
    geometry
    foundation
    sfm
    slam
    bundle
)
set_target_properties(pyslam PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)