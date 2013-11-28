##############################################################################
# pre-defined search paths
##############################################################################
SET(JSON_INCLUDE_SEARCH_DIRS
	${GLOBAL_EXT_DIR}/inc/json
	${GUACAMOLE_EXT_DIR}/inc/json
    /usr/include
)

SET(JSON_LIBRARY_SEARCH_DIRS
	${GLOBAL_EXT_DIR}/lib
	${GUACAMOLE_EXT_DIR}/lib
    /usr/lib
)

##############################################################################
# feedback to provide user-defined paths to search for json
##############################################################################
MACRO (request_json_search_directories)
    
    IF ( NOT JSON_INCLUDE_DIRS AND NOT JSON_LIBRARY_DIRS )
        SET(JSON_INCLUDE_SEARCH_DIR "Please provide json include path." CACHE PATH "path to json headers.")
        SET(JSON_LIBRARY_SEARCH_DIR "Please provide json library path." CACHE PATH "path to json libraries.")
        MESSAGE(FATAL_ERROR "find_json.cmake: unable to find json.")
    ENDIF ( NOT JSON_INCLUDE_DIRS AND NOT JSON_LIBRARY_DIRS )

    IF ( NOT JSON_INCLUDE_DIRS )
        SET(JSON_INCLUDE_SEARCH_DIR "Please provide json include path." CACHE PATH "path to json headers.")
        MESSAGE(FATAL_ERROR "find_json.cmake: unable to find json headers.")
    ELSE ( NOT JSON_INCLUDE_DIRS )
        UNSET(JSON_INCLUDE_SEARCH_DIR CACHE)
    ENDIF ( NOT JSON_INCLUDE_DIRS )

    IF ( NOT JSON_LIBRARY_DIRS )
        SET(JSON_LIBRARY_SEARCH_DIR "Please provide json library path." CACHE PATH "path to json libraries.")
        MESSAGE(FATAL_ERROR "find_json.cmake: unable to find json libraries.")
    ELSE ( NOT JSON_LIBRARY_DIRS )
        UNSET(JSON_LIBRARY_SEARCH_DIR CACHE)
    ENDIF ( NOT JSON_LIBRARY_DIRS ) 

ENDMACRO (request_json_search_directories)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for JSON")

IF (NOT JSON_INCLUDE_DIRS)

    SET(_JSON_FOUND_INC_DIRS "")

    FOREACH(_SEARCH_DIR ${JSON_INCLUDE_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
            NAMES jsoncpp/json/json.h
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _JSON_FOUND_INC_DIRS ${_CUR_SEARCH})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${JSON_INCLUDE_SEARCH_DIRS})

    IF (NOT _JSON_FOUND_INC_DIRS)
        request_json_search_directories()
    ENDIF (NOT _JSON_FOUND_INC_DIRS)
	
	  FOREACH(_INC_DIR ${_JSON_FOUND_INC_DIRS})
        LIST(APPEND _JSON_INCLUDE_DIRS ${_INC_DIR})
    ENDFOREACH(_INC_DIR ${_BOOST_FOUND_INC_DIRS})
    
    IF (_JSON_FOUND_INC_DIRS)
        SET(JSON_INCLUDE_DIRS ${_JSON_INCLUDE_DIRS} CACHE PATH "path to json headers.")
    ENDIF (_JSON_FOUND_INC_DIRS)

ENDIF(NOT JSON_INCLUDE_DIRS)

IF(UNIX)
	SET(JSON_LIB_FILENAME "libjsoncpp.so")
ELSEIF(WIN32)
	SET(JSON_LIB_FILENAME "json.lib")
ENDIF(UNIX)

IF ( JSON_INCLUDE_DIRS AND NOT JSON_LIBRARIES )

    SET(_JSON_FOUND_LIB_DIR "")
    SET(_JSON_POSTFIX "")

    FOREACH(_SEARCH_DIR ${JSON_LIBRARY_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
				NAMES ${JSON_LIB_FILENAME}
                PATHS ${_SEARCH_DIR}
				PATH_SUFFIXES debug release
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _JSON_FOUND_LIB_DIR ${_SEARCH_DIR})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${JSON_LIBRARY_SEARCH_DIRS})

    IF (NOT _JSON_FOUND_LIB_DIR)
      request_json_search_directories()
    ELSE (NOT _JSON_FOUND_LIB_DIR)
        SET(JSON_LIBRARY_DIRS ${_JSON_FOUND_LIB_DIR} CACHE STRING "The json library directory.")
        message(STATUS "--  found matching version")
    ENDIF (NOT _JSON_FOUND_LIB_DIR)
    
    IF (_JSON_FOUND_LIB_DIR)
        SET(JSON_LIBRARIES ${JSON_LIB_FILENAME} CACHE STRING "The json library.")
    ENDIF (_JSON_FOUND_LIB_DIR)
    
ENDIF( JSON_INCLUDE_DIRS AND NOT JSON_LIBRARIES )

##############################################################################
# verify
##############################################################################
IF ( NOT JSON_INCLUDE_DIRS OR NOT JSON_LIBRARY_DIRS )
    request_json_search_directories()
ELSE ( NOT JSON_INCLUDE_DIRS OR NOT JSON_LIBRARY_DIRS )
    UNSET(JSON_INCLUDE_SEARCH_DIR CACHE)
    UNSET(JSON_LIBRARY_SEARCH_DIR CACHE)
    MESSAGE(STATUS "--  found matching json version")
ENDIF ( NOT JSON_INCLUDE_DIRS OR NOT JSON_LIBRARY_DIRS )



