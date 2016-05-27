##############################################################################
# search paths
##############################################################################
SET(OVR_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/LibOVR/include
  ${OVR_INCLUDE_DIRS}
  ${OVR_INCLUDE_SEARCH_DIR}
  /opt/OculusSDK/currentSDK2/LibOVR/Include
  /opt/OculusSDK/currentSDK2/LibOVRKernel/Src
)

SET(OVR_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/LibOVR/lib
  ${OVR_LIBRARY_DIRS}
  ${OVR_LIBRARY_SEARCH_DIR}
  /opt/OculusSDK/currentSDK2/LibOVR/Lib/Linux/x86_64/Release
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for Oculus SDK")

find_path(OVR_INCLUDE_DIR NAMES OVR_CAPI.h Kernel/OVR_Types.h PATHS ${OVR_INCLUDE_SEARCH_DIRS})

IF (MSVC)
	find_library(OVR_LIBRARY NAMES libOVR64.lib libOVR.lib PATHS ${OVR_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
ELSEIF (UNIX)
	find_library(OVR_LIBRARY NAMES libOVRRT64_0.so PATHS ${OVR_LIBRARY_SEARCH_DIRS})
ENDIF (MSVC)

