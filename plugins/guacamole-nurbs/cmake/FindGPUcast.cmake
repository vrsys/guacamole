##############################################################################
# search paths
##############################################################################
SET(GPUCAST_INCLUDE_SEARCH_DIRS
  I:/repositories/gpucast-github/gpucast
  ${GLOBAL_EXT_DIR}/inc/gpucast
  ${GPUCAST_INCLUDE_SEARCH_DIR}
  /usr/include
)

SET(GPUCAST_LIBRARY_SEARCH_DIRS
  I:/repositories/gpucast-github/gpucast/lib
  ${GLOBAL_EXT_DIR}/lib
  ${GPUCAST_LIBRARY_SEARCH_DIR}
  /usr/lib
  /usr/lib/x86_64-linux-gnu
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for GPUCAST")

find_path(GPUCAST_INCLUDE_DIR NAMES gpucast/core/gpucast.hpp PATHS ${GPUCAST_INCLUDE_SEARCH_DIRS} PATH_SUFFIXES gpucast_core/include)
find_path(GPUCAST_MATH_INCLUDE_DIR NAMES gpucast/math/gpucast_math.hpp PATHS ${GPUCAST_INCLUDE_SEARCH_DIRS} PATH_SUFFIXES gpucast_math/include)
find_path(GPUCAST_GL_INCLUDE_DIR NAMES gpucast/gl/glpp.hpp PATHS ${GPUCAST_INCLUDE_SEARCH_DIRS} PATH_SUFFIXES gpucast_gl/include)

IF (MSVC)
	find_library(GPUCAST_LIBRARY_DEBUG NAMES gpucast_core.lib GPUCAST32d.lib PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
  find_library(GPUCAST_LIBRARY_RELEASE NAMES gpucast_core.lib PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  find_path(GPUCAST_RUNTIME_LIBRARY_DIR NAMES release/gpucast_core.dll debug/gpucast_core.dll PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS})
ELSEIF (UNIX)
	find_library(GPUCAST_LIBRARY NAMES libgpucast_core.so PATHS ${GPUCAST_LIBRARY_SEARCH_DIRS})
ENDIF (MSVC)
