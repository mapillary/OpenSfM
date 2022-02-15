file(GLOB VLFEAT_SRCS vl/*.c vl/*.h)

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
