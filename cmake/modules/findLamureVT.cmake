##############################################################################
# search paths
##############################################################################
SET(LAMURE_VT_INCLUDE_SEARCH_DIRS
  ${LAMURE_VT_ROOT}/include
  ${GLOBAL_EXT_DIR}/lamure/include
  ${LAMURE_VT_INCLUDE_SEARCH_DIR}
  /usr/include
  /opt/local
)

SET(LAMURE_VT_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/lamure/lib
  ${LAMURE_VT_LIBRARY_SEARCH_DIR}
  ${LAMURE_VT_ROOT}/lib
  /usr/lib
  /usr/lib/x86_64-linux-gnu
  /opt/local/lib
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for LAMURE")

find_path(LAMURE_VT_COMMON_INCLUDE_DIR NAMES /lamure/utils.h PATHS ${LAMURE_VT_INCLUDE_SEARCH_DIRS})
find_path(LAMURE_VT_RENDERING_INCLUDE_DIR NAMES /lamure/vt/ren/CutUpdate.h PATHS ${LAMURE_VT_INCLUDE_SEARCH_DIRS})
find_path(LAMURE_VT_PREPROCESSING_INCLUDE_DIR NAMES /lamure/vt/pre/Preprocessor.h PATHS ${LAMURE_VT_INCLUDE_SEARCH_DIRS})

IF (MSVC)
  find_library(LAMURE_VT_LIBRARY NAMES lamure_virtual_texturing.lib PATHS ${LAMURE_VT_LIBRARY_SEARCH_DIRS})

  find_library(LAMURE_COMMON_LIBRARY NAMES lamure_common.lib PATHS ${LAMURE_VT_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  find_library(LAMURE_PREPROCESSING_LIBRARY NAMES lamure_preprocessing.lib PATHS ${LAMURE_VT_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  find_library(LAMURE_RENDERING_LIBRARY NAMES lamure_rendering.lib PATHS ${LAMURE_VT_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
  
  #find_library(LAMURE_COMMON_LIBRARY_DEBUG NAMES lamure_commond.lib PATHS ${LAMURE_VT_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
  #find_library(LAMURE_PREPROCESSING_LIBRARY_DEBUG NAMES lamure_preprocessingd.lib PATHS ${LAMURE_VT_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)
  #find_library(LAMURE_RENDERING_LIBRARY_DEBUG NAMES lamure_renderingd.lib PATHS ${LAMURE_VT_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug)

ELSEIF (UNIX)

  find_library(LAMURE_VT_LIBRARY NAMES liblamure_virtual_texturing.so PATHS ${LAMURE_VT_LIBRARY_SEARCH_DIRS})
  #find_library(LAMURE_COMMON_LIBRARY NAMES liblamure_common.so PATHS ${LAMURE_LIBRARY_SEARCH_DIRS})
  #find_library(LAMURE_PREPROCESSING_LIBRARY NAMES liblamure_preprocessing.so PATHS ${LAMURE_LIBRARY_SEARCH_DIRS})
  #find_library(LAMURE_RENDERING_LIBRARY NAMES liblamure_rendering.so PATHS ${LAMURE_LIBRARY_SEARCH_DIRS})

  #set(LAMURE_COMMON_LIBRARY_DEBUG ${LAMURE_COMMON_LIBRARY})
  #set(LAMURE_PREPROCESSING_LIBRARY_DEBUG ${LAMURE_PREPROCESSING_LIBRARY})
  #set(LAMURE_RENDERING_LIBRARY_DEBUG ${LAMURE_RENDERING_LIBRARY})

ENDIF (MSVC)

##############################################################################
# verification
##############################################################################
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LAMURE DEFAULT_MSG
    LAMURE_COMMON_LIBRARY LAMURE_RENDERING_LIBRARY
    LAMURE_COMMON_INCLUDE_DIR LAMURE_RENDERING_INCLUDE_DIR)
find_package_handle_standard_args(LAMURE DEFAULT_MSG
    LAMURE_VT_LIBRARY LAMURE_VT_RENDERING_INCLUDE_DIR)
if(LAMURE_FOUND)
   set(LAMURE_VT_INCLUDE_DIRS ${LAMURE_VT_COMMON_INCLUDE_DIR} ${LAMURE_VT_RENDERING_INCLUDE_DIR})
endif()