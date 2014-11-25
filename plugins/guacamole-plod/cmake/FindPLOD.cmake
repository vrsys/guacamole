##############################################################################
# search paths
##############################################################################
SET(PLOD_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/inc/plod
  ${PLOD_INCLUDE_SEARCH_DIR}
  /usr/include
  /opt/local
)

SET(PLOD_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/lib
  ${PLOD_LIBRARY_SEARCH_DIR}
  /usr/lib
  /usr/lib/x86_64-linux-gnu
  /opt/local/lib
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for PLOD")

find_path(PLOD_INCLUDE_DIR NAMES PLOD/core/PLOD.hpp PATHS ${PLOD_INCLUDE_SEARCH_DIRS})

IF (MSVC)
	find_library(PLOD_LIBRARY_DEBUG NAMES PLOD.lib PLODd.lib PATHS ${PLOD_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES lib/Debug)
  find_library(PLOD_LIBRARY_RELEASE NAMES PLOD.lib PATHS ${PLOD_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES lib/Release)
  find_path(PLOD_RUNTIME_LIBRARY_DIR NAMES Release/PLOD.dll PATHS ${PLOD_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES bin)
ELSEIF (UNIX)
	find_library(PLOD_LIBRARY NAMES libPLOD.so PATHS ${PLOD_LIBRARY_SEARCH_DIRS})
ENDIF (MSVC)
