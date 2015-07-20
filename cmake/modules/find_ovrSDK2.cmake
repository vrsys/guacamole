##############################################################################
# search paths
##############################################################################
SET(OVRSDK2_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/inc/ovr/include
  ${OVRSDK2_INCLUDE_DIRS}
  ${OVRSDK2_INCLUDE_SEARCH_DIR}
  /opt/OculusSDK/currentSDK2/LibOVR/Include
  /opt/OculusSDK/currentSDK2/LibOVRKernel/Src
)

SET(OVRSDK2_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/lib
  ${OVRSDK2_LIBRARY_DIRS}
  ${OVRSDK2_LIBRARY_SEARCH_DIR}
  /opt/OculusSDK/currentSDK2/LibOVR/Lib/Linux/x86_64/Release
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for OculusSDK2")

find_path(OVRSDK2_INCLUDE_DIR NAMES OVR.h Kernel/OVR_Types.h PATHS ${OVRSDK2_INCLUDE_SEARCH_DIRS})

IF (MSVC)
	find_library(OVRSDK2_LIBRARY NAMES libovr64.lib PATHS ${OVRSDK2_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
	find_library(OVRSDK2_LIBRARY_DEBUG NAMES libovr64.lib PATHS ${OVRSDK2_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
ELSEIF (UNIX)
	find_library(OVRSDK2_LIBRARY NAMES libovr.a PATHS ${OVRSDK2_LIBRARY_SEARCH_DIRS})
ENDIF (MSVC)

