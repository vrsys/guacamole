##############################################################################
# search paths
##############################################################################
SET(GPUCAST_INCLUDE_SEARCH_DIRS
  I:/repositories/gpucast-install/include
  ${GLOBAL_EXT_DIR}/inc/gpucast
  ${GPUCAST_INCLUDE_SEARCH_DIR}
  /usr/include
  /opt/local
)

SET(GPUCAST_LIBRARY_SEARCH_DIRS
  I:/repositories/gpucast-install
  ${GLOBAL_EXT_DIR}/lib
  ${GPUCAST_LIBRARY_SEARCH_DIR}
  /usr/lib
  /usr/lib/x86_64-linux-gnu
  /opt/local/lib
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for GPUCAST")

find_path(GPUCAST_INCLUDE_DIR NAMES gpucast/core/gpucast.hpp PATHS ${GPUCAST_INCLUDE_SEARCH_DIRS})

IF (MSVC)
	find_library(GPUCAST_LIBRARY_DEBUG NAMES gpucast_core.lib GPUCAST32d.lib PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES lib/Debug)
  find_library(GPUCAST_LIBRARY_RELEASE NAMES gpucast_core.lib PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES lib/Release)
  find_path(GPUCAST_RUNTIME_LIBRARY_DIR NAMES Release/gpucast_core.dll PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES bin)
ELSEIF (UNIX)
	find_library(GPUCAST_LIBRARY NAMES libgpucast_core.so PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS})
ENDIF (MSVC)
