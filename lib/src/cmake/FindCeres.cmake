# - Find Ceres library
# Find the native Ceres includes and library
# This module defines
#  CERES_INCLUDE_DIRS, where to find ceres.h, Set when
#                      CERES_INCLUDE_DIR is found.
#  CERES_LIBRARIES, libraries to link against to use Ceres.
#  CERES_ROOT_DIR, The base directory to search for Ceres.
#                  This can also be an environment variable.
#  CERES_FOUND, If false, do not try to use Ceres.
#
# also defined, but not for general use are
#  CERES_LIBRARY, where to find the Ceres library.

# If CERES_ROOT_DIR was defined in the environment, use it.
IF(NOT CERES_ROOT_DIR AND NOT $ENV{CERES_ROOT_DIR} STREQUAL "")
  SET(CERES_ROOT_DIR $ENV{CERES_ROOT_DIR})
ENDIF()

SET(_ceres_SEARCH_DIRS
  ${CERES_ROOT_DIR}
  /usr/local
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt/lib/ceres
)

FIND_PATH(CERES_INCLUDE_DIR
  NAMES
    ceres/ceres.h
  HINTS
    ${_ceres_SEARCH_DIRS}
  PATH_SUFFIXES
    include
)

FIND_LIBRARY(CERES_LIBRARY
  NAMES
    ceres
  HINTS
    ${_ceres_SEARCH_DIRS}
  PATH_SUFFIXES
    lib64 lib
  )

# handle the QUIETLY and REQUIRED arguments and set CERES_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ceres DEFAULT_MSG
    CERES_LIBRARY CERES_INCLUDE_DIR)

IF(CERES_FOUND)
  SET(CERES_LIBRARIES ${CERES_LIBRARY})
  SET(CERES_INCLUDE_DIRS ${CERES_INCLUDE_DIR})
ENDIF(CERES_FOUND)

MARK_AS_ADVANCED(
  CERES_INCLUDE_DIR
  CERES_LIBRARY
)
