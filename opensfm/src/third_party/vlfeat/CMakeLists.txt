file(GLOB VLFEAT_SRCS vl/*.c vl/*.h)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-absolute-value")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-logical-not-parentheses")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-tautological-constant-out-of-range-compare")

if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" )
    add_definitions( -DVL_DISABLE_SSE2 )
endif()

if(WIN32)
    add_definitions(-D__SSE2__)
endif()

add_library(vl ${VLFEAT_SRCS})


target_include_directories(vl
  PRIVATE
    ${CMAKE_SOURCE_DIR}/third_party/vlfeat
    ${OpenMP_CXX_INCLUDE_DIR})
