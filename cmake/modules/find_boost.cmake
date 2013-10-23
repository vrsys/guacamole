SET(GUA_BOOST_MIN_VERSION_MAJOR 1)
SET(GUA_BOOST_MIN_VERSION_MINOR 46)
SET(GUA_BOOST_MIN_VERSION_SUBMINOR 0)
SET(GUA_BOOST_MIN_VERSION "${GUA_BOOST_MIN_VERSION_MAJOR}.${GUA_BOOST_MIN_VERSION_MINOR}.${GUA_BOOST_MIN_VERSION_SUBMINOR}")
MATH(EXPR GUA_BOOST_MIN_VERSION_NUM "${GUA_BOOST_MIN_VERSION_MAJOR}*10000 + ${GUA_BOOST_MIN_VERSION_MINOR}*100 + ${GUA_BOOST_MIN_VERSION_SUBMINOR}")

SET(GUA_BOOST_INCLUDE_SEARCH_DIRS
    ${GLOBAL_EXT_DIR}/inc/boost
    /opt/boost/latest/include
    ${CMAKE_SYSTEM_INCLUDE_PATH}
    ${CMAKE_INCLUDE_PATH}
    /usr/include
)

SET(GUA_BOOST_LIBRARY_SEARCH_DIRS
    ${GLOBAL_EXT_DIR}/lib
    /opt/boost/latest/lib
    ${CMAKE_SYSTEM_LIBRARY_PATH}
    ${CMAKE_LIBRARY_PATH}
    /usr/lib64
)

message("-- checking for BOOST")

IF (NOT BOOST_INCLUDE_DIRS)

    SET(_GUA_BOOST_FOUND_INC_DIRS "")
    FOREACH(_SEARCH_DIR ${GUA_BOOST_INCLUDE_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
                NAMES boost/config.hpp
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _GUA_BOOST_FOUND_INC_DIRS ${_CUR_SEARCH})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${GUA_BOOST_INCLUDE_SEARCH_DIRS})

    IF (NOT _GUA_BOOST_FOUND_INC_DIRS)
        MESSAGE(FATAL_ERROR "guacamole_boost.cmake: unable to find boost library")
	ELSE (NOT _GUA_BOOST_FOUND_INC_DIRS)
		SET(BOOST_INCLUDE_DIRS ${_GUA_BOOST_FOUND_INC_DIRS})
    ENDIF (NOT _GUA_BOOST_FOUND_INC_DIRS)

    SET(GUA_BOOST_VERSION       0)
    SET(GUA_BOOST_LIB_VERSION   "")
    SET(GUA_BOOST_LIB_SUFFIX    "")

    SET(_GUA_BOOST_CUR_VERSION ${GUA_BOOST_MIN_VERSION_NUM})

    FOREACH(_INC_DIR ${_GUA_BOOST_FOUND_INC_DIRS})
        FILE(READ "${_INC_DIR}/boost/version.hpp" _GUA_BOOST_VERSION_CONTENTS)

        STRING(REGEX REPLACE ".*#define BOOST_VERSION ([0-9]+).*" "\\1"             _BOOST_VERSION       "${_GUA_BOOST_VERSION_CONTENTS}")
        STRING(REGEX REPLACE ".*#define BOOST_LIB_VERSION \"([0-9_]+)\".*" "\\1"    _BOOST_LIB_VERSION   "${_GUA_BOOST_VERSION_CONTENTS}")

        IF(NOT "${_BOOST_VERSION}" STREQUAL "0")
            MATH(EXPR _BOOST_MAJOR_VERSION      "${_BOOST_VERSION} / 100000")
            MATH(EXPR _BOOST_MINOR_VERSION      "${_BOOST_VERSION} / 100 % 1000")
            MATH(EXPR _BOOST_SUBMINOR_VERSION   "${_BOOST_VERSION} % 100")
        ELSE (NOT "${_BOOST_VERSION}" STREQUAL "0")
            MESSAGE("WTF")
        ENDIF(NOT "${_BOOST_VERSION}" STREQUAL "0")

        MATH(EXPR _BOOST_VERSION_NUM    "${_BOOST_MAJOR_VERSION}*10000  + ${_BOOST_MINOR_VERSION}*100   + ${_BOOST_SUBMINOR_VERSION}")

        IF (   _GUA_BOOST_CUR_VERSION LESS  _BOOST_VERSION_NUM
            OR _GUA_BOOST_CUR_VERSION EQUAL _BOOST_VERSION_NUM)
            SET(GUA_BOOST_VERSION               ${_BOOST_VERSION})
            SET(GUA_BOOST_LIB_VERSION           ${_BOOST_LIB_VERSION})
            SET(GUA_BOOST_LIB_SUFFIX            ".${_BOOST_MAJOR_VERSION}.${_BOOST_MINOR_VERSION}.${_BOOST_SUBMINOR_VERSION}")
            SET(BOOST_INCLUDE_DIRS               ${_INC_DIR})
            SET(_GUA_BOOST_CUR_VERSION          ${_BOOST_VERSION_NUM})
        ENDIF (   _GUA_BOOST_CUR_VERSION LESS  _BOOST_VERSION_NUM
               OR _GUA_BOOST_CUR_VERSION EQUAL _BOOST_VERSION_NUM)

    ENDFOREACH(_INC_DIR ${_GUA_BOOST_FOUND_INC_DIRS})

    IF (GUA_BOOST_VERSION EQUAL 0)
        MESSAGE(FATAL_ERROR "found boost versions ${_BOOST_VERSION} to old (min. version ${GUA_BOOST_MIN_VERSION} required)")
    ELSE (GUA_BOOST_VERSION EQUAL 0)
      #SET(BOOST_INCLUDE_DIRS               ${BOOST_INCLUDE_DIRS}               CACHE STRING "The boost include directory")
        SET(BOOST_INCLUDE_DIRS               ${BOOST_INCLUDE_DIRS})
        SET(GUA_BOOST_VERSION               ${GUA_BOOST_VERSION}               CACHE STRING "The boost version number")
        SET(GUA_BOOST_LIB_SUFFIX            ${GUA_BOOST_LIB_SUFFIX}            CACHE STRING "The boost library suffix")
        SET(GUA_BOOST_LIB_VERSION           ${GUA_BOOST_LIB_VERSION}           CACHE STRING "The boost library version string")
    ENDIF (GUA_BOOST_VERSION EQUAL 0)

ENDIF(NOT BOOST_INCLUDE_DIRS)

IF (        BOOST_INCLUDE_DIRS
    AND NOT BOOST_LIBRARIES)
    #AND NOT BOOST_LIBRARY_DIRS)

    SET(_GUA_BOOST_FILESYSTEM_LIB "")
    SET(_GUA_BOOST_FOUND_LIB_DIR "")
    SET(_GUA_BOOST_POSTFIX "")

    if (UNIX)
        LIST(APPEND _GUA_BOOST_FILESYSTEM_LIB   "${CMAKE_SHARED_LIBRARY_PREFIX}boost_filesystem${GUA_COMPILER_SUFFIX}${GUA_BOOST_LIB_VERSION}${CMAKE_SHARED_LIBRARY_SUFFIX}${GUA_BOOST_LIB_SUFFIX}"
                                                "${CMAKE_SHARED_LIBRARY_PREFIX}boost_filesystem${CMAKE_SHARED_LIBRARY_SUFFIX}${GUA_BOOST_LIB_SUFFIX}"
                                                "${CMAKE_SHARED_LIBRARY_PREFIX}boost_filesystem${CMAKE_SHARED_LIBRARY_SUFFIX}${GUA_BOOST_LIB_SUFFIX}")
    elseif (WIN32)
        LIST(APPEND _GUA_BOOST_FILESYSTEM_LIB   "${CMAKE_STATIC_LIBRARY_PREFIX}libboost_filesystem-${GUA_COMPILER_SUFFIX}-mt-${GUA_BOOST_LIB_VERSION}${CMAKE_STATIC_LIBRARY_SUFFIX}"
                                                "${CMAKE_STATIC_LIBRARY_PREFIX}libboost_filesystem-mt${CMAKE_STATIC_LIBRARY_SUFFIX}")
    endif (UNIX)


    FOREACH(_SEARCH_DIR ${GUA_BOOST_LIBRARY_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
                NAMES ${_GUA_BOOST_FILESYSTEM_LIB}
                PATHS ${_SEARCH_DIR}
                PATH_SUFFIXES debug release .
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _GUA_BOOST_FOUND_LIB_DIR ${_SEARCH_DIR})
            if (UNIX)
                FIND_FILE(_CUR_SEARCH_FILE NAMES ${_GUA_BOOST_FILESYSTEM_LIB} PATHS ${_CUR_SEARCH})
                if (_CUR_SEARCH_FILE)
                    LIST(APPEND BOOST_LIBRARIES ${_CUR_SEARCH_FILE})
                    STRING(REGEX REPLACE "${_CUR_SEARCH}/${CMAKE_SHARED_LIBRARY_PREFIX}boost_filesystem(.*).so(.*)" "\\2" _GUA_BOOST_UNIX_LIB_SUF ${_CUR_SEARCH_FILE})
                    if (${_GUA_BOOST_UNIX_LIB_SUF} STREQUAL ${GUA_BOOST_LIB_SUFFIX})
                        message("found matching version")
                        list(APPEND BOOST_LIBRARIES _GUA_BOOST_FILESYSTEM_LIB)
                        STRING(REGEX REPLACE "${_CUR_SEARCH}/${CMAKE_SHARED_LIBRARY_PREFIX}boost_filesystem(.*).so(.*)" "\\1" _GUA_BOOST_POSTFIX ${_CUR_SEARCH_FILE})
                    endif (${_GUA_BOOST_UNIX_LIB_SUF} STREQUAL ${GUA_BOOST_LIB_SUFFIX})
                endif (_CUR_SEARCH_FILE)
                SET(_CUR_SEARCH_FILE _CUR_SEARCH_FILE-NOTFOUND CACHE INTERNAL "internal use")
            endif (UNIX)
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${GUA_BOOST_LIBRARY_SEARCH_DIRS})

    IF (NOT _GUA_BOOST_FOUND_LIB_DIR)
        MESSAGE(FATAL_ERROR "guacamole_boost.cmake: unable to find boost library")
    ELSE (NOT _GUA_BOOST_FOUND_LIB_DIR)
        SET(BOOST_LIBRARY_DIRS       ${_GUA_BOOST_FOUND_LIB_DIR}       CACHE STRING "The boost library directory")
		message("--  found matching version")
                FILE(GLOB BOOST_LIBRARIES ${_GUA_BOOST_FOUND_LIB_DIR}/*.so)
    ENDIF (NOT _GUA_BOOST_FOUND_LIB_DIR)

    if (UNIX)
#            SET(SCHISM_BOOST_LIB_POSTFIX_REL    "${_GUA_BOOST_POSTFIX}"     CACHE STRING "(deprecated) boost library release postfix")
#            SET(SCHISM_BOOST_LIB_POSTFIX_DBG    "${_GUA_BOOST_POSTFIX}"     CACHE STRING "(deprecated) boost library debug postfix")
#        IF (NOT _GUA_BOOST_POSTFIX)
#            MESSAGE(FATAL_ERROR "guacamole_boost.cmake: unable to determine boost library suffix")
#        ELSE (NOT _GUA_BOOST_POSTFIX)
#            SET(SCHISM_BOOST_LIB_POSTFIX_REL    "${_GUA_BOOST_POSTFIX}"     CACHE STRING "boost library release postfix")
#            SET(SCHISM_BOOST_LIB_POSTFIX_DBG    "${_GUA_BOOST_POSTFIX}"     CACHE STRING "boost library debug postfix")
#        ENDIF (NOT _GUA_BOOST_POSTFIX)
        set(GUA_BOOST_MT_REL   "${_GUA_BOOST_POSTFIX}" CACHE STRING "boost dynamic library release postfix")
        set(GUA_BOOST_MT_DBG   "${_GUA_BOOST_POSTFIX}" CACHE STRING "boost dynamic library debug postfix")
        set(GUA_BOOST_MT_S_REL "${_GUA_BOOST_POSTFIX}" CACHE STRING "boost static library release postfix")
        set(GUA_BOOST_MT_S_DBG "${_GUA_BOOST_POSTFIX}" CACHE STRING "boost static library debug postfix")
    elseif (WIN32)
#        SET(SCHISM_BOOST_LIB_POSTFIX_REL    "${GUA_COMPILER_SUFFIX}-mt-${GUA_BOOST_LIB_VERSION}"     CACHE STRING "(deprecated) boost library release postfix")
#        SET(SCHISM_BOOST_LIB_POSTFIX_DBG    "${GUA_COMPILER_SUFFIX}-mt-gd-${GUA_BOOST_LIB_VERSION}"  CACHE STRING "(deprecated) boost library debug postfix")

        set(GUA_BOOST_MT_REL    "${GUA_COMPILER_SUFFIX}-mt-${GUA_BOOST_LIB_VERSION}"     CACHE STRING "boost dynamic library release postfix")
        set(GUA_BOOST_MT_DBG    "${GUA_COMPILER_SUFFIX}-mt-gd-${GUA_BOOST_LIB_VERSION}"  CACHE STRING "boost dynamic library debug postfix")
        set(GUA_BOOST_MT_S_REL  "${GUA_COMPILER_SUFFIX}-mt-s-${GUA_BOOST_LIB_VERSION}"   CACHE STRING "boost static library release postfix")
        set(GUA_BOOST_MT_S_DBG  "${GUA_COMPILER_SUFFIX}-mt-sgd-${GUA_BOOST_LIB_VERSION}" CACHE STRING "boost static library debug postfix")
    endif (UNIX)
	
ENDIF(BOOST_INCLUDE_DIRS AND NOT BOOST_LIBRARIES)
#      AND NOT BOOST_LIBRARY_DIRS)
