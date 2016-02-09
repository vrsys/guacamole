##############################################################################
# search paths
##############################################################################
SET(SCHISM_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/schism/include
  ${SCHISM_INCLUDE_SEARCH_DIR}
  /opt/schism/current
)

SET(SCHISM_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/schism/lib
  ${SCHISM_LIBRARY_SEARCH_DIR}
  ../
  /opt/schism/current/lib/linux_x86
)

##############################################################################
# feedback to provide user-defined paths to search for schism
##############################################################################
MACRO (request_schism_search_directories)

    IF ( NOT SCHISM_INCLUDE_DIRS AND NOT SCHISM_LIBRARY_DIRS )
        SET(SCHISM_INCLUDE_SEARCH_DIR "Please provide schism include path." CACHE PATH "path to schism headers.")
        SET(SCHISM_LIBRARY_SEARCH_DIR "Please provide schism library path." CACHE PATH "path to schism libraries.")
        MESSAGE(FATAL_ERROR "find_schism.cmake: unable to find schism.")
    ENDIF ( NOT SCHISM_INCLUDE_DIRS AND NOT SCHISM_LIBRARY_DIRS )

    IF ( NOT SCHISM_INCLUDE_DIRS )
        SET(SCHISM_INCLUDE_SEARCH_DIR "Please provide schism include path." CACHE PATH "path to schism headers.")
        MESSAGE(FATAL_ERROR "find_schism.cmake: unable to find schism headers.")
    ELSE ( NOT SCHISM_INCLUDE_DIRS )
        UNSET(SCHISM_INCLUDE_SEARCH_DIR CACHE)
    ENDIF ( NOT SCHISM_INCLUDE_DIRS )

    IF ( NOT SCHISM_LIBRARY_DIRS )
        SET(SCHISM_LIBRARY_SEARCH_DIR "Please provide schism library path." CACHE PATH "path to schism libraries.")
        MESSAGE(FATAL_ERROR "find_schism.cmake: unable to find schism libraries.")
    ELSE ( NOT SCHISM_LIBRARY_DIRS )
        UNSET(SCHISM_LIBRARY_SEARCH_DIR CACHE)
    ENDIF ( NOT SCHISM_LIBRARY_DIRS )

ENDMACRO (request_schism_search_directories)

##############################################################################
# check for schism
##############################################################################
message(STATUS "-- checking for schism")

IF ( NOT SCHISM_INCLUDE_DIRS )

    FOREACH(_SEARCH_DIR ${SCHISM_INCLUDE_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
                NAMES scm_gl_core/src/scm/gl_core.h
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _SCHISM_FOUND_INC_DIRS ${_CUR_SEARCH})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${SCHISM_INCLUDE_SEARCH_DIRS})

    IF (NOT _SCHISM_FOUND_INC_DIRS)
        request_schism_search_directories()
    ENDIF (NOT _SCHISM_FOUND_INC_DIRS)

    FOREACH(_INC_DIR ${_SCHISM_FOUND_INC_DIRS})
        LIST(APPEND _SCHISM_INCLUDE_DIRS ${_INC_DIR}/scm_cl_core/src)
        LIST(APPEND _SCHISM_INCLUDE_DIRS ${_INC_DIR}/scm_core/src)
        LIST(APPEND _SCHISM_INCLUDE_DIRS ${_INC_DIR}/scm_gl_core/src)
        LIST(APPEND _SCHISM_INCLUDE_DIRS ${_INC_DIR}/scm_gl_util/src)
        LIST(APPEND _SCHISM_INCLUDE_DIRS ${_INC_DIR}/scm_input/src)
    ENDFOREACH(_INC_DIR ${_BOOST_FOUND_INC_DIRS})

    IF (_SCHISM_FOUND_INC_DIRS)
        SET(SCHISM_INCLUDE_DIRS ${_SCHISM_INCLUDE_DIRS} CACHE PATH "path to schism headers.")
    ENDIF (_SCHISM_FOUND_INC_DIRS)

ENDIF ( NOT SCHISM_INCLUDE_DIRS )

IF(MSVC)
  SET(SCHISM_DEBUG_SUFFIX -gd)
ENDIF()

FIND_LIBRARY(SCHISM_CORE_LIBRARY NAMES libscm_core scm_core HINTS ${SCHISM_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release NO_DEFAULT_PATH)
FIND_LIBRARY(SCHISM_GL_CORE_LIBRARY NAMES libscm_gl_core scm_gl_core PATHS  ${SCHISM_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release NO_DEFAULT_PATH)
FIND_LIBRARY(SCHISM_GL_UTIL_LIBRARY NAMES libscm_gl_util scm_gl_util PATHS ${SCHISM_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release NO_DEFAULT_PATH)
FIND_LIBRARY(SCHISM_INPUT_LIBRARY NAMES libscm_input scm_input PATHS ${SCHISM_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES release NO_DEFAULT_PATH)

FIND_LIBRARY(SCHISM_CORE_LIBRARY_DEBUG NAMES libscm_core${SCHISM_DEBUG_SUFFIX} scm_core${SCHISM_DEBUG_SUFFIX} PATHS ${SCHISM_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug NO_DEFAULT_PATH)
FIND_LIBRARY(SCHISM_GL_CORE_LIBRARY_DEBUG NAMES libscm_gl_core${SCHISM_DEBUG_SUFFIX} scm_gl_core${SCHISM_DEBUG_SUFFIX} PATHS ${SCHISM_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug NO_DEFAULT_PATH)
FIND_LIBRARY(SCHISM_GL_UTIL_LIBRARY_DEBUG  NAMES libscm_gl_util${SCHISM_DEBUG_SUFFIX} scm_gl_util${SCHISM_DEBUG_SUFFIX} PATHS ${SCHISM_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug NO_DEFAULT_PATH)
FIND_LIBRARY(SCHISM_INPUT_LIBRARY_DEBUG NAMES libscm_input${SCHISM_DEBUG_SUFFIX} scm_input${SCHISM_DEBUG_SUFFIX} PATHS ${SCHISM_LIBRARY_SEARCH_DIRS} PATH_SUFFIXES debug NO_DEFAULT_PATH)

##############################################################################
# verify
##############################################################################
IF(SCHISM_INCLUDE_DIRS AND SCHISM_CORE_LIBRARY AND SCHISM_GL_CORE_LIBRARY AND SCHISM_GL_UTIL_LIBRARY AND SCHISM_INPUT_LIBRARY)
  message(STATUS "Found schism.")
ELSE()
  request_schism_search_directories()
ENDIF()

