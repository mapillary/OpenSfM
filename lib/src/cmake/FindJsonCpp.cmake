# - Try to find JsonCpp
#
# The following variables are optionally searched for defaults
#  JSONCPP_ROOT_DIR:            Base directory where all JsonCpp components are found
#
# The following are set after configuration is done: 
#  JSONCPP_FOUND
#  JSONCPP_INCLUDE_DIRS
#  JSONCPP_LIBRARIES
#  JSONCPP_LIBRARY_DIRS

include(FindPackageHandleStandardArgs)

set(JSONCPP_ROOT_DIR $ENV{JSONCPP_DIR} CACHE PATH "Folder contains Google JSONCPP")

if(WIN32)
    find_path(JSONCPP_INCLUDE_DIR json/json.h
        PATHS ${JSONCPP_ROOT_DIR}/include)
else()
    find_path(JSONCPP_INCLUDE_DIR json/json.h
        PATHS ${JSONCPP_ROOT_DIR} $ENV{CPLUS_INCLUDE_DIR} /usr/include
        PATH_SUFFIXES jsoncpp)
    message(${JSONCPP_INCLUDE_DIR})
endif()

if(MSVC)
    find_library(JSONCPP_LIBRARY_RELEASE jsoncpp.lib
        PATHS ${JSONCPP_ROOT_DIR}
        PATH_SUFFIXES lib)

    find_library(JSONCPP_LIBRARY_DEBUG jsoncppd.lib
        PATHS ${JSONCPP_ROOT_DIR}
        PATH_SUFFIXES lib)

    set(JSONCPP_LIBRARY optimized ${JSONCPP_LIBRARY_RELEASE} debug ${JSONCPP_LIBRARY_DEBUG})
else()
    find_library(JSONCPP_LIBRARY libjsoncpp.so
        PATHS ${JSONCPP_ROOT_DIR} $ENV{LD_LIBRARY_PATH} /usr
        PATH_SUFFIXES lib lib64)
endif()

find_package_handle_standard_args(JSONCPP DEFAULT_MSG
    JSONCPP_INCLUDE_DIR JSONCPP_LIBRARY)

if(JSONCPP_FOUND)
    set(JSONCPP_INCLUDE_DIRS ${JSONCPP_INCLUDE_DIR})
    set(JSONCPP_LIBRARIES ${JSONCPP_LIBRARY})
endif()

