##############################################################################
# search paths
##############################################################################
SET(LAMURE_INCLUDE_SEARCH_DIRS
  /home/ephtron/Projects/lamure/install/include
  ${LAMURE_ROOT}/include
  ${GLOBAL_EXT_DIR}/lamure/include
  ${LAMURE_INCLUDE_SEARCH_DIR}
  ${LAMURE_ROOT}/include
  /usr/include
  /opt/local  
  ../lamure/install/include
)

SET(LAMURE_LIBRARY_SEARCH_DIRS
  /home/ephtron/Projects/lamure/install/lib
  ${GLOBAL_EXT_DIR}/lamure/lib
  ${LAMURE_LIBRARY_SEARCH_DIR}
  ${LAMURE_ROOT}/lib
  /usr/lib
  /usr/lib/x86_64-linux-gnu
  /opt/local/lib
  ../lamure/install/lib
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for LAMURE")

find_path(LAMURE_COMMON_INCLUDE_DIR NAMES /lamure/utils.h PATHS ${LAMURE_INCLUDE_SEARCH_DIRS})
find_path(LAMURE_RENDERING_INCLUDE_DIR NAMES /lamure/ren/cut_update_queue.h PATHS ${LAMURE_INCLUDE_SEARCH_DIRS})
find_path(LAMURE_PREPROCESSING_INCLUDE_DIR NAMES /lamure/pre/reduction_normal_deviation_clustering.h PATHS ${LAMURE_INCLUDE_SEARCH_DIRS})
find_path(LAMURE_PROVENANCE_INCLUDE_DIR NAMES /lamure/prov/auxi.h PATHS ${LAMURE_INCLUDE_SEARCH_DIRS})

IF (MSVC)

  find_library(LAMURE_COMMON_LIBRARY NAMES lamure_common.lib PATHS ${LAMURE_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  find_library(LAMURE_PREPROCESSING_LIBRARY NAMES lamure_preprocessing.lib PATHS ${LAMURE_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  find_library(LAMURE_RENDERING_LIBRARY NAMES lamure_rendering.lib PATHS ${LAMURE_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  find_library(LAMURE_PROVENANCE_LIBRARY NAMES lamure_provenance.lib PATHS ${LAMURE_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)

  find_library(LAMURE_COMMON_LIBRARY_DEBUG NAMES lamure_commond.lib PATHS ${LAMURE_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
  find_library(LAMURE_PREPROCESSING_LIBRARY_DEBUG NAMES lamure_preprocessingd.lib PATHS ${LAMURE_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
  find_library(LAMURE_RENDERING_LIBRARY_DEBUG NAMES lamure_renderingd.lib PATHS ${LAMURE_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
  find_library(LAMURE_PROVENANCE_LIBRARY_DEBUG NAMES lamure_provenanced.lib PATHS ${LAMURE_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)


ELSEIF (UNIX)

  find_library(LAMURE_COMMON_LIBRARY NAMES liblamure_common.so PATHS ${LAMURE_LIBRARY_SEARCH_DIRS})
  find_library(LAMURE_PREPROCESSING_LIBRARY NAMES liblamure_preprocessing.so PATHS ${LAMURE_LIBRARY_SEARCH_DIRS})
  find_library(LAMURE_RENDERING_LIBRARY NAMES liblamure_rendering.so PATHS ${LAMURE_LIBRARY_SEARCH_DIRS})
  find_library(LAMURE_PROVENANCE_LIBRARY NAMES liblamure_provenance.so PATHS ${LAMURE_LIBRARY_SEARCH_DIRS})
  

  set(LAMURE_COMMON_LIBRARY_DEBUG ${LAMURE_COMMON_LIBRARY})
  set(LAMURE_PREPROCESSING_LIBRARY_DEBUG ${LAMURE_PREPROCESSING_LIBRARY})
  set(LAMURE_RENDERING_LIBRARY_DEBUG ${LAMURE_RENDERING_LIBRARY})
  set(LAMURE_PROVENANCE_LIBRARY_DEBUG ${LAMURE_PROVENANCE_LIBRARY})

ENDIF (MSVC)

##############################################################################
# verification
##############################################################################
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LAMURE DEFAULT_MSG
    LAMURE_COMMON_LIBRARY LAMURE_RENDERING_LIBRARY
    LAMURE_COMMON_INCLUDE_DIR LAMURE_RENDERING_INCLUDE_DIR LAMURE_PROVENANCE_INCLUDE_DIR)

if(LAMURE_FOUND)
	set(LAMURE_INCLUDE_DIRS ${LAMURE_COMMON_INCLUDE_DIR} ${LAMURE_RENDERING_INCLUDE_DIR} ${LAMURE_PROVENANCE_INCLUDE_DIR})
endif()
