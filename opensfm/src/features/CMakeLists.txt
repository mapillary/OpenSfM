set(FEATURES_FILES
    akaze_bind.h
    hahog.h
    matching.h
    src/akaze_bind.cc
    src/hahog.cc
    src/matching.cc
)
add_library(features ${FEATURES_FILES})
target_link_libraries(features
  PRIVATE
    ${OpenCV_LIBS}
    akaze
    foundation
    vl
)
target_include_directories(features PRIVATE ${CMAKE_SOURCE_DIR})

pybind11_add_module(pyfeatures python/pybind.cc)
target_include_directories(pyfeatures PRIVATE ${GLOG_INCLUDE_DIR})
target_link_libraries(pyfeatures
  PRIVATE
    features
    foundation
    pybind11
    akaze
)
set_target_properties(pyfeatures PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)
