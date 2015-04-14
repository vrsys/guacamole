##############################################################################
# search paths
##############################################################################
SET(OVR_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/inc/ovr/include
  ${OVR_INCLUDE_DIRS}
  ${OVR_INCLUDE_SEARCH_DIR}
  /opt/OculusSDK/current/LibOVR/Include
  /opt/OculusSDK/current/LibOVRKernel/Src
)

SET(OVR_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/lib
  ${OVR_LIBRARY_DIRS}
  ${OVR_LIBRARY_SEARCH_DIR}
  /opt/OculusSDK/current/LibOVR/Lib/Linux/Release/x86_64
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for OculusSDK")

find_path(OVR_INCLUDE_DIR NAMES OVR.h Kernel/OVR_Types.h PATHS ${OVR_INCLUDE_SEARCH_DIRS})
message(${OVR_INCLUDE_DIR})

IF (MSVC)
	find_library(OVR_LIBRARY NAMES libovr64.lib PATHS ${OVR_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
	find_library(OVR_LIBRARY_DEBUG NAMES libovr64.lib PATHS ${OVR_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
ELSEIF (UNIX)
	find_library(OVR_LIBRARY NAMES libovr.a PATHS ${OVR_LIBRARY_SEARCH_DIRS})
ENDIF (MSVC)

