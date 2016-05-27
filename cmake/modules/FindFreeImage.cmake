##############################################################################
# search paths
##############################################################################
SET(FREEIMAGE_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/freeimage/include
  ${FREEIMAGE_INCLUDE_SEARCH_DIR}
  /usr/include
)

SET(FREEIMAGE_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/freeimage/lib
  ${FREEIMAGE_LIBRARY_SEARCH_DIR}
  /usr/lib
  /usr/lib/x86_64-linux-gnu
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for FREEIMAGE")

find_path(FREEIMAGE_INCLUDE_DIR NAMES FreeImage.h PATHS ${FREEIMAGE_INCLUDE_SEARCH_DIRS})

IF (MSVC)
	find_library(FREEIMAGE_LIBRARY NAMES FreeImage.lib PATHS ${FREEIMAGE_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  find_library(FREEIMAGE_PLUS_LIBRARY NAMES FreeImagePlus.lib PATHS ${FREEIMAGE_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
 
  find_library(FREEIMAGE_LIBRARY_DEBUG NAMES FreeImage.lib PATHS ${FREEIMAGE_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
  find_library(FREEIMAGE_PLUS_LIBRARY_DEBUG NAMES FreeImagePlus.lib FREEIMAGE3d.lib PATHS ${FREEIMAGE_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
  
  get_filename_component(_FREEIMAGE_LIBRARY_DIR ${FREEIMAGE_LIBRARY} DIRECTORY)
  get_filename_component(_FREEIMAGE_LIBRARY_DEBUG_DIR ${FREEIMAGE_LIBRARY_DEBUG} DIRECTORY)
  set(FREEIMAGE_LIBRARY_DIR ${_FREEIMAGE_LIBRARY_DIR} ${_FREEIMAGE_LIBRARY_DEBUG_DIR} CACHE STRING "freeimage library path.")
ELSEIF (UNIX)
	#find_library(FREEIMAGE_LIBRARY NAMES libFREEIMAGE3.a PATHS ${FREEIMAGE_LIBRARY_SEARCH_DIRS})
ENDIF (MSVC)
