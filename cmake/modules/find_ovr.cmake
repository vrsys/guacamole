##############################################################################
# search paths
##############################################################################
SET(OVR_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/inc/ovr/include
  ${OVR_INCLUDE_DIRS}
  ${OVR_INCLUDE_SEARCH_DIR}
  /opt/OculusSDK/LibOVR/Include
)

SET(OVR_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/lib
  ${OVR_LIBRARY_DIRS}
  ${OVR_LIBRARY_SEARCH_DIR}
  /opt/OculusSDK/LibOVR/Lib/Linux/Release/x86_64
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for OculusSDK")

find_path(OVR_INCLUDE_DIR NAMES OVR.h PATHS ${OVR_INCLUDE_SEARCH_DIRS})

IF (MSVC)
	find_library(OVR_LIBRARY_RELEASE NAMES libovr64.lib PATHS ${OVR_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
	find_library(OVR_LIBRARY_DEBUG NAMES libovr64.lib PATHS ${OVR_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
ELSEIF (UNIX)
	find_library(OVR_LIBRARY NAMES libovr.a PATHS ${OVR_LIBRARY_SEARCH_DIRS})
ENDIF (MSVC)

