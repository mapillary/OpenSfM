set(AKAZE_SRCS
    lib/AKAZEConfig.h
    lib/AKAZE.h
    lib/AKAZE.cpp
    lib/fed.h
    lib/fed.cpp
    lib/nldiffusion_functions.h
    lib/nldiffusion_functions.cpp
    lib/utils.h
    lib/utils.cpp
)
add_library(akaze ${AKAZE_SRCS})
target_link_libraries(akaze ${OpenCV_LIBS})
target_include_directories(akaze
  PUBLIC
    ${CMAKE_SOURCE_DIR}/third_party/akaze/lib
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${OpenMP_CXX_INCLUDE_DIR})
