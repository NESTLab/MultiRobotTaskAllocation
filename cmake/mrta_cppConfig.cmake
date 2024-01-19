# .rst:
# CRA_CAFConfig
# --------
#
# Find Module for CRA CAF
#
# This modules finds if CRA CAF is installed and determines where the
# tools, include files and libraries are.
#
# This module set the following result variables:
#
# ::
#
#   MRTA_CPP_LIBRARY     = The full path of the MRTA_CPP library
#   MRTA_CPP_INCLUDE_DIR = The full path to the .h include files
#
# Examples Usages:
#
# ::
#
#   find_package(MRTA_CPP)
#   if(MRTA_CPP_FOUND)
#     includee_directories(${MRTA_CPP_INCLUDE_DIR})
#     ...
#     target_link_libraries(... ${MRTA_CPP_LIBRARY})
#   endif(MRTA_CPP_FOUND)
#
#   find_package(MRTA_CPP REQUIRED)
#   include_directories(${MRTA_CPP_INCLUDE_DIR})
#   ...
#   target_link_libraries(... ${MRTA_CPP_LIBRARY})


#
# Standard MRTA-CPP library paths
#
set(_MRTA_CPP_LIBRARY_PATHS
  /usr/lib
  /usr/local/lib
  /opt/lib
  /opt/local/lib)

#
# Standard MRTA-CPP C include paths
#
set(_MRTA_CPP_INCLUDE_PATHS
  /usr/include
  /usr/local/include
  /opt/include
  /opt/local/include)

#
# Look for MRTA-CPP library
#
find_library(MRTA_CPP_LIBRARY
  NAMES mrta_utilities
  PATHS ${_MRTA_CPP_LIBRARY_PATHS}
  DOC "Location of the MRTA-CPP library")

#
# Look for MRTA-CPP include files
#
find_path(MRTA_CPP_INCLUDE_DIR
  NAMES mrta_utilities/mrta_config.h
  PATHS ${_MRTA_CPP_INCLUDE_PATHS}
  DOC "Location of the MRTA-CPP C include files")

set(MRTA_CPP_FOUND 0)
if(MRTA_CPP_LIBRARY AND MRTA_CPP_INCLUDE_DIR)
  set(MRTA_CPP_FOUND 1)
endif(MRTA_CPP_LIBRARY AND MRTA_CPP_INCLUDE_DIR)
if(NOT QUIET)
  if(MRTA_CPP_FOUND)
    message(STATUS "MRTA-CPP library found: ${MRTA_CPP_LIBRARY}")
    message(STATUS "MRTA-CPP headers include path found: ${MRTA_CPP_INCLUDE_DIR}")
  else(MRTA_CPP_FOUND)
    message(STATUS "MRTA-CPP not found")
  endif(MRTA_CPP_FOUND)
endif(NOT QUIET)
set(MRTA_CPP_FOUND ${MRTA_CPP_FOUND} CACHE BOOL "Whether MRTA-CPP was found")

mark_as_advanced(MRTA_CPP_LIBRARY MRTA_CPP_INCLUDE_DIR)
