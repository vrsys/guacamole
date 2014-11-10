##############################################################################
# search paths
##############################################################################
SET(AWESOMIUM_INCLUDE_SEARCH_DIRS
  ${AWESOMIUM_INCLUDE_DIRS}
  ${AWESOMIUM_INCLUDE_SEARCH_DIR}
  /opt/awesomium/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../awesomium/include
)

SET(AWESOMIUM_LIBRARY_SEARCH_DIRS
  ${AWESOMIUM_LIBRARY_DIRS}
  ${AWESOMIUM_LIBRARY_SEARCH_DIR}
  /opt/awesomium/bin
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../awesomium/bin
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for Awesomium")

find_path(AWESOMIUM_INCLUDE_DIR NAMES Awesomium/Platform.h PATHS ${AWESOMIUM_INCLUDE_SEARCH_DIRS})

IF (MSVC)
  # find_library(OVR_LIBRARY_RELEASE NAMES ovr.lib PATHS ${AWESOMIUM_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  # find_library(OVR_LIBRARY_DEBUG NAMES ovr.lib ovrd.lib PATHS ${AWESOMIUM_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
ELSEIF (UNIX)
  find_library(AWESOMIUM_LIBRARY NAMES libawesomium-1-7.so.4.0 PATHS ${AWESOMIUM_LIBRARY_SEARCH_DIRS})
ENDIF (MSVC)

