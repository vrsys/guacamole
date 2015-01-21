##############################################################################
# search paths
##############################################################################
SET(PBR_INCLUDE_SEARCH_DIRS
  ${PBR_ROOT}/include
  ${GLOBAL_EXT_DIR}/inc/pbr
  ${PBR_INCLUDE_SEARCH_DIR}
  ${PBR_ROOT}/common/include
  ${PBR_ROOT}/common/src
  ${PBR_ROOT}/preprocessing/include
  ${PBR_ROOT}/preprocessing/src
  ${PBR_ROOT}/rendering/include
  ${PBR_ROOT}/rendering/src
  /usr/include
  /opt/local  
)

SET(PBR_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/lib
  ${PBR_LIBRARY_SEARCH_DIR}
  ${PBR_ROOT}/lib
  /usr/lib
  /usr/lib/x86_64-linux-gnu
  /opt/local/lib
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for PBR")

find_path(PBR_COMMON_INCLUDE_DIR NAMES pbr/utils.h PATHS ${PBR_INCLUDE_SEARCH_DIRS})
find_path(PBR_RENDERING_INCLUDE_DIR NAMES pbr/ren/cut_update_queue.h PATHS ${PBR_INCLUDE_SEARCH_DIRS})
find_path(PBR_PREPROCESSING_INCLUDE_DIR NAMES pbr/pre/reduction_normal_deviation_clustering.h PATHS ${PBR_INCLUDE_SEARCH_DIRS})

IF (MSVC)

	find_library(PBR_COMMON_LIBRARY NAMES pbr_common.lib PATHS ${PBR_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  find_library(PBR_PREPROCESSING_LIBRARY NAMES pbr_preprocessing.lib PATHS ${PBR_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  find_library(PBR_RENDERING_LIBRARY NAMES pbr_rendering.lib PATHS ${PBR_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  
  find_library(PBR_COMMON_LIBRARY_DEBUG NAMES pbr_common.lib PATHS ${PBR_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
  find_library(PBR_PREPROCESSING_LIBRARY_DEBUG NAMES pbr_preprocessing.lib PATHS ${PBR_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
  find_library(PBR_RENDERING_LIBRARY_DEBUG NAMES pbr_rendering.lib PATHS ${PBR_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)

ELSEIF (UNIX)

	find_library(PBR_COMMON_LIBRARY NAMES libpbr_common.so PATHS ${PBR_LIBRARY_SEARCH_DIRS})
  find_library(PBR_PREPROCESSING_LIBRARY NAMES libpbr_preprocessing.so PATHS ${PBR_LIBRARY_SEARCH_DIRS})
  find_library(PBR_RENDERING_LIBRARY NAMES libpbr_rendering.so PATHS ${PBR_LIBRARY_SEARCH_DIRS})

  set(PBR_COMMON_LIBRARY_DEBUG ${PBR_COMMON_LIBRARY})
  set(PBR_PREPROCESSING_LIBRARY_DEBUG ${PBR_PREPROCESSING_LIBRARY})
  set(PBR_RENDERING_LIBRARY_DEBUG ${PBR_RENDERING_LIBRARY})

ENDIF (MSVC)

##############################################################################
# verification
##############################################################################
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PBR DEFAULT_MSG
    PBR_COMMON_LIBRARY PBR_RENDERING_LIBRARY
    PBR_COMMON_INCLUDE_DIR PBR_RENDERING_INCLUDE_DIR)

if(PBR_FOUND)
   set(PBR_INCLUDE_DIRS ${PBR_COMMON_INCLUDE_DIR} ${PBR_RENDERING_INCLUDE_DIR})
endif()
