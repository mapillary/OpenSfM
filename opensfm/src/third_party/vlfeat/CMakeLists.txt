file(GLOB VLFEAT_SRCS vl/*.c vl/*.h)
add_library(vl ${VLFEAT_SRCS})
target_include_directories(vl
  PRIVATE
    ${CMAKE_SOURCE_DIR}/third_party/vlfeat
    ${OpenMP_CXX_INCLUDE_DIR})
