set(BUNDLE_FILES
    bundle_adjuster.h
    reconstruction_alignment.h
    src/error_utils.h
    src/motion_prior_terms.h
    src/absolute_motion_terms.h
    src/relative_motion_terms.h
    src/bundle_adjuster.cc
)
add_library(bundle ${BUNDLE_FILES})
target_link_libraries(bundle 
  PRIVATE
    ${CERES_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${SUITESPARSE_LIBRARIES}
    foundation
)
if (LAPACK_FOUND)
    target_include_directories(bundle PRIVATE ${LAPACK_INCLUDE_DIRS})
endif()
if (SUITESPARSE_FOUND)
    target_include_directories(bundle PRIVATE ${SUITESPARSE_INCLUDE_DIRS})
endif()
target_include_directories(bundle PRIVATE ${CMAKE_SOURCE_DIR})

pybind11_add_module(pybundle python/pybind.cc)
target_link_libraries(pybundle PRIVATE
  bundle
  geometry
  foundation
  pybind11)
set_target_properties(pybundle PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${opensfm_SOURCE_DIR}/.."
)