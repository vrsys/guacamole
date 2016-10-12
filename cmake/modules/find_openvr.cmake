##############################################################################
# search paths
##############################################################################
SET(OPENVR_INCLUDE_SEARCH_DIRS
    ${GLOBAL_EXT_DIR}/openvr/include
    ${OPENVR_INCLUDE_DIRS}
    ${OPENVR_INCLUDE_SEARCH_DIR}
)

SET(OPENVR_LIBRARY_SEARCH_DIRS
    ${GLOBAL_EXT_DIR}/openvr/lib
    ${OPENVR_LIBRARY_DIRS}
    ${OPENVR_LIBRARY_SEARCH_DIR}
)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for Open VR")

find_path(OPENVR_INCLUDE_DIR NAMES openvr.h PATHS ${OPENVR_INCLUDE_SEARCH_DIRS})

IF (MSVC)
    find_library(OPENVR_LIBRARY NAMES win64/openvr_api.lib win32/openvr_api.lib PATHS ${OPENVR_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release)
ELSEIF (UNIX)
    find_library(OPENVR_LIBRARY NAMES linux64/libopenvr_api.so linux32/libopenvr_api.so PATHS ${OPENVR_LIBRARY_SEARCH_DIRS})
ENDIF (MSVC)

