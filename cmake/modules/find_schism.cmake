##############################################################################
# search paths
##############################################################################
SET(SCHISM_INCLUDE_SEARCH_DIRS
  ${SCHISM_INCLUDE_DIRS}
  ${SCHISM_INCLUDE_SEARCH_DIR}
  /opt/schism/current
)

SET(SCHISM_LIBRARY_SEARCH_DIRS
  ${SCHISM_LIBRARY_DIRS}
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

IF ( SCHISM_INCLUDE_DIRS AND ( NOT SCHISM_LIBRARY_DIRS OR NOT SCHISM_LIBRARIES))

    FOREACH(_SEARCH_DIR ${SCHISM_LIBRARY_SEARCH_DIRS})

		    IF (UNIX)
			    FIND_PATH(_CUR_SEARCH
					    NAMES libscm_gl_core.so
					    PATHS ${_SEARCH_DIR}
					    NO_DEFAULT_PATH)
		    ELSEIF(WIN32)
			    FIND_PATH(_CUR_SEARCH
					    NAMES scm_gl_core.lib
					    PATHS ${_SEARCH_DIR}
					    PATH_SUFFIXES debug release
					    NO_DEFAULT_PATH)
		    ENDIF(UNIX)

        IF (_CUR_SEARCH)
            LIST(APPEND _SCHISM_FOUND_LIB_DIR ${_SEARCH_DIR})
        ENDIF(_CUR_SEARCH)

        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")

    ENDFOREACH(_SEARCH_DIR ${SCHISM_LIBRARY_SEARCH_DIRS})

    IF (NOT _SCHISM_FOUND_LIB_DIR)
        request_schism_search_directories()
    ELSE (NOT _SCHISM_FOUND_LIB_DIR)
		    SET(SCHISM_LIBRARY_DIRS ${_SCHISM_FOUND_LIB_DIR} CACHE STRING "The schism library directory.")
    ENDIF (NOT _SCHISM_FOUND_LIB_DIR)

    SET(_SCHISM_LIBRARIES "")

    FOREACH(_LIB_DIR ${_SCHISM_FOUND_LIB_DIR})
		    IF (UNIX)
			    file(GLOB _SCHISM_LIBRARIES ${_LIB_DIR}/*.so)
		    ELSEIF(WIN32)
			    file(GLOB _SCHISM_LIBRARY_ABSOLUTE_PATHS ${_LIB_DIR}/release/scm*.lib)
			    FOREACH (_SCHISM_LIB_PATH ${_SCHISM_LIBRARY_ABSOLUTE_PATHS})
				    SET(_SCHISM_LIB_FILENAME "")
				    GET_FILENAME_COMPONENT(_SCHISM_LIB_FILENAME ${_SCHISM_LIB_PATH} NAME)
				    LIST(APPEND _SCHISM_LIBRARIES ${_SCHISM_LIB_FILENAME})
			    ENDFOREACH(_SCHISM_LIB_PATH)
		    ENDIF(UNIX)
    ENDFOREACH(_LIB_DIR ${_SCHISM_FOUND_INC_DIRS})

    IF (_SCHISM_FOUND_LIB_DIR)
        SET(SCHISM_LIBRARIES ${_SCHISM_LIBRARIES} CACHE STRING "schism libraries.")
    ENDIF (_SCHISM_FOUND_LIB_DIR)

ENDIF ( SCHISM_INCLUDE_DIRS AND ( NOT SCHISM_LIBRARY_DIRS OR NOT SCHISM_LIBRARIES))

##############################################################################
# verify
##############################################################################
IF ( NOT SCHISM_INCLUDE_DIRS OR NOT SCHISM_LIBRARY_DIRS )
    request_schism_search_directories()
ELSE ( NOT SCHISM_INCLUDE_DIRS OR NOT SCHISM_LIBRARY_DIRS )
    UNSET(SCHISM_INCLUDE_SEARCH_DIR CACHE)
    UNSET(SCHISM_LIBRARY_SEARCH_DIR CACHE)
    MESSAGE(STATUS "--  found matching schism version")
ENDIF ( NOT SCHISM_INCLUDE_DIRS OR NOT SCHISM_LIBRARY_DIRS )



