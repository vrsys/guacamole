SET(BOOST_MIN_VERSION_MAJOR 1)
SET(BOOST_MIN_VERSION_MINOR 46)
SET(BOOST_MIN_VERSION_SUBMINOR 0)
SET(BOOST_MIN_VERSION "${BOOST_MIN_VERSION_MAJOR}.${BOOST_MIN_VERSION_MINOR}.${BOOST_MIN_VERSION_SUBMINOR}")
MATH(EXPR BOOST_MIN_VERSION_NUM "${BOOST_MIN_VERSION_MAJOR}*10000 + ${BOOST_MIN_VERSION_MINOR}*100 + ${BOOST_MIN_VERSION_SUBMINOR}")

SET(BOOST_INCLUDE_SEARCH_DIRS
    ${GLOBAL_EXT_DIR}/inc/boost
    /opt/boost/latest/include
    ${CMAKE_SYSTEM_INCLUDE_PATH}
    ${CMAKE_INCLUDE_PATH}
    /usr/include
)

SET(BOOST_LIBRARY_SEARCH_DIRS
    ${GLOBAL_EXT_DIR}/lib
    /opt/boost/latest/lib
    ${CMAKE_SYSTEM_LIBRARY_PATH}
    ${CMAKE_LIBRARY_PATH}
    /usr/lib64
)

message("-- checking for boost")

IF (NOT BOOST_INCLUDE_DIRS)

    SET(_BOOST_FOUND_INC_DIRS "")

    FOREACH(_SEARCH_DIR ${BOOST_INCLUDE_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
                NAMES boost/config.hpp
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _BOOST_FOUND_INC_DIRS ${_CUR_SEARCH})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${BOOST_INCLUDE_SEARCH_DIRS})

    IF (NOT _BOOST_FOUND_INC_DIRS)
        MESSAGE(FATAL_ERROR "find_boost.cmake: unable to find boost include directory")
    ENDIF (NOT _BOOST_FOUND_INC_DIRS)

    SET(BOOST_VERSION       0)
    SET(BOOST_LIB_VERSION   "")
    SET(BOOST_LIB_SUFFIX    "")

    SET(_BOOST_CUR_VERSION ${BOOST_MIN_VERSION_NUM})

    FOREACH(_INC_DIR ${_BOOST_FOUND_INC_DIRS})
        FILE(READ "${_INC_DIR}/boost/version.hpp" _BOOST_VERSION_CONTENTS)

        STRING(REGEX REPLACE ".*#define BOOST_VERSION ([0-9]+).*" "\\1"             _BOOST_VERSION       "${_BOOST_VERSION_CONTENTS}")
        STRING(REGEX REPLACE ".*#define BOOST_LIB_VERSION \"([0-9_]+)\".*" "\\1"    _BOOST_LIB_VERSION   "${_BOOST_VERSION_CONTENTS}")

        IF(NOT "${_BOOST_VERSION}" STREQUAL "0")
            MATH(EXPR _BOOST_MAJOR_VERSION      "${_BOOST_VERSION} / 100000")
            MATH(EXPR _BOOST_MINOR_VERSION      "${_BOOST_VERSION} / 100 % 1000")
            MATH(EXPR _BOOST_SUBMINOR_VERSION   "${_BOOST_VERSION} % 100")
        ELSE (NOT "${_BOOST_VERSION}" STREQUAL "0")
            MESSAGE("WTF")
        ENDIF(NOT "${_BOOST_VERSION}" STREQUAL "0")

        MATH(EXPR _BOOST_VERSION_NUM    "${_BOOST_MAJOR_VERSION}*10000  + ${_BOOST_MINOR_VERSION}*100   + ${_BOOST_SUBMINOR_VERSION}")

        IF (   _BOOST_CUR_VERSION LESS  _BOOST_VERSION_NUM
            OR _BOOST_CUR_VERSION EQUAL _BOOST_VERSION_NUM)
            SET(BOOST_VERSION               ${_BOOST_VERSION})
            SET(BOOST_LIB_VERSION           ${_BOOST_LIB_VERSION})
            SET(BOOST_LIB_SUFFIX            ".${_BOOST_MAJOR_VERSION}.${_BOOST_MINOR_VERSION}.${_BOOST_SUBMINOR_VERSION}")
            SET(BOOST_INCLUDE_DIRS               ${_INC_DIR})
            SET(_BOOST_CUR_VERSION          ${_BOOST_VERSION_NUM})
        ENDIF (   _BOOST_CUR_VERSION LESS  _BOOST_VERSION_NUM
               OR _BOOST_CUR_VERSION EQUAL _BOOST_VERSION_NUM)

    ENDFOREACH(_INC_DIR ${_BOOST_FOUND_INC_DIRS})

    IF (BOOST_VERSION EQUAL 0)
        MESSAGE(FATAL_ERROR "found boost versions ${_BOOST_VERSION} to old (min. version ${BOOST_MIN_VERSION} required)")
    ELSE (BOOST_VERSION EQUAL 0)
        SET(BOOST_INCLUDE_DIRS               ${BOOST_INCLUDE_DIRS}               CACHE STRING "The boost include directory")
        SET(BOOST_VERSION               ${BOOST_VERSION}               CACHE STRING "The boost version number")
        SET(BOOST_LIB_SUFFIX            ${BOOST_LIB_SUFFIX}            CACHE STRING "The boost library suffix")
        SET(BOOST_LIB_VERSION           ${BOOST_LIB_VERSION}           CACHE STRING "The boost library version string")
    ENDIF (BOOST_VERSION EQUAL 0)

ENDIF(NOT BOOST_INCLUDE_DIRS)

IF (        BOOST_INCLUDE_DIRS
    AND NOT BOOST_LIBRARIES)

    SET(_BOOST_FILESYSTEM_LIB "")
    SET(_BOOST_FOUND_LIB_DIR "")
    SET(_BOOST_POSTFIX "")

    FOREACH(_SEARCH_DIR ${BOOST_LIBRARY_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
                NAMES libboost_filesystem.so
                PATHS ${_SEARCH_DIR}
                PATH_SUFFIXES debug release
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _BOOST_FOUND_LIB_DIR ${_SEARCH_DIR})
            if (UNIX)
                FIND_FILE(_CUR_SEARCH_FILE NAMES ${_BOOST_FILESYSTEM_LIB} PATHS ${_CUR_SEARCH})
                if (_CUR_SEARCH_FILE)
                    STRING(REGEX REPLACE "${_CUR_SEARCH}/${CMAKE_SHARED_LIBRARY_PREFIX}boost_filesystem(.*).so(.*)" "\\2" _BOOST_UNIX_LIB_SUF ${_CUR_SEARCH_FILE})
                    if (${_BOOST_UNIX_LIB_SUF} STREQUAL ${BOOST_LIB_SUFFIX})
                        message("--  found matching version")
                        STRING(REGEX REPLACE "${_CUR_SEARCH}/${CMAKE_SHARED_LIBRARY_PREFIX}boost_filesystem(.*).so(.*)" "\\1" _BOOST_POSTFIX ${_CUR_SEARCH_FILE})
                    endif (${_BOOST_UNIX_LIB_SUF} STREQUAL ${BOOST_LIB_SUFFIX})
                endif (_CUR_SEARCH_FILE)
                SET(_CUR_SEARCH_FILE _CUR_SEARCH_FILE-NOTFOUND CACHE INTERNAL "internal use")
            endif (UNIX)
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${BOOST_LIBRARY_SEARCH_DIRS})

    IF (NOT _BOOST_FOUND_LIB_DIR)
        MESSAGE(FATAL_ERROR "schism_boost.cmake: unable to find boost library")
    ELSE (NOT _BOOST_FOUND_LIB_DIR)
        file(GLOB BOOST_LIBRARIES ${_BOOST_FOUND_LIB_DIR}/*.so)
    ENDIF (NOT _BOOST_FOUND_LIB_DIR)

    if (UNIX)
        set(BOOST_MT_REL   "${_BOOST_POSTFIX}" CACHE STRING "boost dynamic library release postfix")
        set(BOOST_MT_DBG   "${_BOOST_POSTFIX}" CACHE STRING "boost dynamic library debug postfix")
        set(BOOST_MT_S_REL "${_BOOST_POSTFIX}" CACHE STRING "boost static library release postfix")
        set(BOOST_MT_S_DBG "${_BOOST_POSTFIX}" CACHE STRING "boost static library debug postfix")
    elseif (WIN32)
        set(BOOST_MT_REL    "${COMPILER_SUFFIX}-mt-${BOOST_LIB_VERSION}"     CACHE STRING "boost dynamic library release postfix")
        set(BOOST_MT_DBG    "${COMPILER_SUFFIX}-mt-gd-${BOOST_LIB_VERSION}"  CACHE STRING "boost dynamic library debug postfix")
        set(BOOST_MT_S_REL  "${COMPILER_SUFFIX}-mt-s-${BOOST_LIB_VERSION}"   CACHE STRING "boost static library release postfix")
        set(BOOST_MT_S_DBG  "${COMPILER_SUFFIX}-mt-sgd-${BOOST_LIB_VERSION}" CACHE STRING "boost static library debug postfix")
    endif (UNIX)

ENDIF(        BOOST_INCLUDE_DIRS
      AND NOT BOOST_LIBRARIES)



