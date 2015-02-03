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

set(JSONCPP_ROOT_DIR $ENV{JSONCPP_DIR} CACHE PATH "Folder contains JSONCPP")

find_path(JSONCPP_INCLUDE_DIR
        json/json.h
    HINTS
        ${JSONCPP_ROOT_DIR}/include
        ENV CPLUS_INCLUDE_PATH
    PATH_SUFFIXES
        jsoncpp
)

find_library(JSONCPP_LIBRARY
    NAMES
        jsoncpp json
    HINTS
        ${JSONCPP_ROOT_DIR}
        ENV LD_LIBRARY_PATH
    PATH_SUFFIXES
        lib64 lib
)

find_package_handle_standard_args(JSONCPP DEFAULT_MSG
    JSONCPP_INCLUDE_DIR JSONCPP_LIBRARY)

if(JSONCPP_FOUND)
    set(JSONCPP_INCLUDE_DIRS ${JSONCPP_INCLUDE_DIR})
    set(JSONCPP_LIBRARIES ${JSONCPP_LIBRARY})
endif()

