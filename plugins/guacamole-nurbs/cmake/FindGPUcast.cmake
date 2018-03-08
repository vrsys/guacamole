##############################################################################
# search paths
##############################################################################
SET(GPUCAST_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/gpucast/include
  ${GPUCAST_INCLUDE_SEARCH_DIR}
  /usr/include
  /opt/gpucast/include
)

SET(GPUCAST_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/gpucast/lib
  ${GPUCAST_LIBRARY_SEARCH_DIR}
  /usr/lib
  /usr/lib/x86_64-linux-gnu
  /opt/gpucast/lib
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for GPUCAST")

find_path(GPUCAST_INCLUDE_DIR NAMES gpucast/core/gpucast.hpp PATHS ${GPUCAST_INCLUDE_SEARCH_DIRS})

IF (MSVC)
	find_library(GPUCAST_LIBRARY_DEBUG NAMES gpucast_cored.lib PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES Debug)
  find_library(GPUCAST_LIBRARY_RELEASE NAMES gpucast_core.lib PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES Release)
  find_path(GPUCAST_RUNTIME_LIBRARY_DIR NAMES Release/gpucast_core.dll PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES bin)
ELSEIF (UNIX)
	find_library(GPUCAST_LIBRARY_RELEASE NAMES libgpucast_core.so PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS})
  find_library(GPUCAST_LIBRARY_DEBUG NAMES libgpucast_core.so PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS})
ENDIF (MSVC)
