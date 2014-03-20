##############################################################################
# search paths
##############################################################################
SET(ASSIMP_INCLUDE_SEARCH_DIRS
	${GLOBAL_EXT_DIR}/inc/assimp
    /opt/assimp/current
    /usr/include
)

SET(ASSIMP_LIBRARY_SEARCH_DIRS
	${GLOBAL_EXT_DIR}/lib
    /opt/assimp/current/assimp-build
    /usr/lib
)

##############################################################################
# feedback to provide user-defined paths to search for assimp
##############################################################################
MACRO (request_assimp_search_directories)

    IF ( NOT ASSIMP_INCLUDE_DIRS AND NOT ASSIMP_LIBRARY_DIRS )
        SET(ASSIMP_INCLUDE_SEARCH_DIR "Please provide assimp include path." CACHE PATH "path to assimp headers.")
        SET(ASSIMP_LIBRARY_SEARCH_DIR "Please provide assimp library path." CACHE PATH "path to assimp libraries.")
        MESSAGE(FATAL_ERROR "find_assimp.cmake: unable to find assimp.")
    ENDIF ( NOT ASSIMP_INCLUDE_DIRS AND NOT ASSIMP_LIBRARY_DIRS )

    IF ( NOT ASSIMP_INCLUDE_DIRS )
        SET(ASSIMP_INCLUDE_SEARCH_DIR "Please provide assimp include path." CACHE PATH "path to assimp headers.")
        MESSAGE(FATAL_ERROR "find_assimp.cmake: unable to find assimp headers.")
    ELSE ( NOT ASSIMP_INCLUDE_DIRS )
        UNSET(ASSIMP_INCLUDE_SEARCH_DIR CACHE)
    ENDIF ( NOT ASSIMP_INCLUDE_DIRS )

    IF ( NOT ASSIMP_LIBRARY_DIRS )
        SET(ASSIMP_LIBRARY_SEARCH_DIR "Please provide assimp library path." CACHE PATH "path to assimp libraries.")
        MESSAGE(FATAL_ERROR "find_assimp.cmake: unable to find assimp libraries.")
    ELSE ( NOT ASSIMP_LIBRARY_DIRS )
        UNSET(ASSIMP_LIBRARY_SEARCH_DIR CACHE)
    ENDIF ( NOT ASSIMP_LIBRARY_DIRS )

ENDMACRO (request_assimp_search_directories)

##############################################################################
# check for assimp
##############################################################################
message(STATUS "-- checking for ASSIMP")

IF (NOT ASSIMP_INCLUDE_DIRS)

    SET(_ASSIMP_FOUND_INC_DIRS "")

    FOREACH(_SEARCH_DIR ${ASSIMP_INCLUDE_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
            NAMES assimp/assimp.h
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _ASSIMP_FOUND_INC_DIRS ${_CUR_SEARCH})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${ASSIMP_INCLUDE_SEARCH_DIRS})

    IF (NOT _ASSIMP_FOUND_INC_DIRS)
        request_assimp_search_directories()
    ENDIF (NOT _ASSIMP_FOUND_INC_DIRS)

    FOREACH(_INC_DIR ${_ASSIMP_FOUND_INC_DIRS})
        LIST(APPEND _ASSIMP_INCLUDE_DIRS ${_INC_DIR})
    ENDFOREACH(_INC_DIR ${_BOOST_FOUND_INC_DIRS})

	IF (_ASSIMP_FOUND_INC_DIRS)
        SET(ASSIMP_INCLUDE_DIRS ${_ASSIMP_INCLUDE_DIRS} CACHE PATH "path to assimp headers.")
    ENDIF (_ASSIMP_FOUND_INC_DIRS)

ENDIF(NOT ASSIMP_INCLUDE_DIRS)


IF(UNIX)
	SET(ASSIMP_LIB_FILENAME "libassimp.so")
ELSEIF(WIN32)
	SET(ASSIMP_LIB_FILENAME "assimp.lib")
ENDIF(UNIX)


IF ( ASSIMP_INCLUDE_DIRS AND ( NOT ASSIMP_LIBRARY_DIRS OR NOT ASSIMP_LIBRARIES))

    SET(_ASSIMP_FOUND_LIB_DIR "")
    SET(_ASSIMP_POSTFIX "")

    FOREACH(_SEARCH_DIR ${ASSIMP_LIBRARY_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
				NAMES ${ASSIMP_LIB_FILENAME}
                PATHS ${_SEARCH_DIR}
				PATH_SUFFIXES debug release
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _ASSIMP_FOUND_LIB_DIR ${_SEARCH_DIR})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${ASSIMP_LIBRARY_SEARCH_DIRS})

    IF (NOT _ASSIMP_FOUND_LIB_DIR)
        request_assimp_search_directories()
    ELSE (NOT _ASSIMP_FOUND_LIB_DIR)
		SET(ASSIMP_LIBRARY_DIRS ${_ASSIMP_FOUND_LIB_DIR} CACHE STRING "The assimp library directory")
    ENDIF (NOT _ASSIMP_FOUND_LIB_DIR)

    FOREACH(_LIB_DIR ${_ASSIMP_FOUND_LIB_DIR})
        LIST(APPEND _ASSIMP_LIBRARIES ${ASSIMP_LIB_FILENAME})
    ENDFOREACH(_LIB_DIR ${_ASSIMP_FOUND_INC_DIRS})

	IF (_ASSIMP_FOUND_LIB_DIR)
        SET(ASSIMP_LIBRARIES ${_ASSIMP_LIBRARIES} CACHE PATH "path to assimp library.")
    ENDIF (_ASSIMP_FOUND_LIB_DIR)

ENDIF ( ASSIMP_INCLUDE_DIRS AND ( NOT ASSIMP_LIBRARY_DIRS OR NOT ASSIMP_LIBRARIES))

##############################################################################
# verify
##############################################################################
IF ( NOT ASSIMP_INCLUDE_DIRS OR NOT ASSIMP_LIBRARY_DIRS )
    request_assimp_search_directories()
ELSE ( NOT ASSIMP_INCLUDE_DIRS OR NOT ASSIMP_LIBRARY_DIRS )
    UNSET(ASSIMP_INCLUDE_SEARCH_DIR CACHE)
    UNSET(ASSIMP_LIBRARY_SEARCH_DIR CACHE)
    MESSAGE(STATUS "--  found matching assimp version")
ENDIF ( NOT ASSIMP_INCLUDE_DIRS OR NOT ASSIMP_LIBRARY_DIRS )

