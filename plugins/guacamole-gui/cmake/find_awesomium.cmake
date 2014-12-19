##############################################################################
# search paths
##############################################################################
SET(AWESOMIUM_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/inc/awesomium
  ${AWESOMIUM_INCLUDE_DIRS}
  ${AWESOMIUM_INCLUDE_SEARCH_DIR}
  /opt/Awesomium/include
)

SET(AWESOMIUM_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/lib
  ${AWESOMIUM_LIBRARY_DIRS}
  ${AWESOMIUM_LIBRARY_SEARCH_DIR}
  /opt/Awesomium/lib
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for Awesomium")

find_path(AWESOMIUM_INCLUDE_DIR NAMES Awesomium/Platform.h PATHS ${AWESOMIUM_INCLUDE_SEARCH_DIRS})

IF (MSVC)
  find_library(AWESOMIUM_LIBRARY NAMES awesomium.lib PATHS ${AWESOMIUM_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  find_library(AWESOMIUM_LIBRARY_DEBUG NAMES awesomium.lib PATHS ${AWESOMIUM_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
ELSEIF (UNIX)
  find_library(AWESOMIUM_LIBRARY NAMES libawesomium-1-7.so PATHS ${AWESOMIUM_LIBRARY_SEARCH_DIRS})
ENDIF (MSVC)

