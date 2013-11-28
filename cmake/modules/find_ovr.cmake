##############################################################################
# search paths
##############################################################################
SET(OVR_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/inc/OculusSDK
  ${OVR_INCLUDE_DIRS}
  ${OVR_INCLUDE_SEARCH_DIR}
  "/opt/OculusSDK/LibOVR/Include"
)

SET(OVR_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/lib
  ${OVR_LIBRARY_DIRS}
  ${OVR_LIBRARY_SEARCH_DIR}
  "/opt/OculusSDK/LibOVR/Lib"
)

##############################################################################
# feedback to provide user-defined paths to search for python
##############################################################################
MACRO (request_ovr_search_directories)
    
    IF ( NOT OVR_INCLUDE_DIRS AND NOT OVR_LIBRARY_DIRS )
        SET(OVR_INCLUDE_SEARCH_DIR "Please provide Oculus SDK include path." CACHE PATH "path to Oculus SDK headers.")
        SET(OVR_LIBRARY_SEARCH_DIR "Please provide Oculus SDK library path." CACHE PATH "path to Oculus SDK libraries.")
        MESSAGE(FATAL_ERROR "find_ovr.cmake: unable to find Oculus SDK.")
    ENDIF ( NOT OVR_INCLUDE_DIRS AND NOT OVR_LIBRARY_DIRS )

    IF ( NOT OVR_INCLUDE_DIRS )
        SET(OVR_INCLUDE_SEARCH_DIR "Please provide Oculus SDK include path." CACHE PATH "path to Oculus SDK headers.")
        MESSAGE(FATAL_ERROR "find_ovr.cmake: unable to find Oculus SDK headers.")
    ELSE ( NOT OVR_INCLUDE_DIRS )
        UNSET(OVR_INCLUDE_SEARCH_DIR CACHE)
    ENDIF ( NOT OVR_INCLUDE_DIRS )

    IF ( NOT OVR_LIBRARY_DIRS )
        SET(OVR_LIBRARY_SEARCH_DIR "Please provide Oculus SDK library path." CACHE PATH "path to Oculus SDK libraries.")
        MESSAGE(FATAL_ERROR "find_ovr.cmake: unable to find Oculus SDK libraries.")
    ELSE ( NOT OVR_LIBRARY_DIRS )
        UNSET(OVR_LIBRARY_SEARCH_DIR CACHE)
    ENDIF ( NOT OVR_LIBRARY_DIRS ) 

ENDMACRO (request_ovr_search_directories)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for OVR")

IF (NOT OVR_INCLUDE_DIRS)

    SET(_OVR_FOUND_INC_DIRS "")
    FOREACH(_SEARCH_DIR ${OVR_INCLUDE_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
            NAMES OVR.h
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _OVR_FOUND_INC_DIRS ${_CUR_SEARCH})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${OVR_INCLUDE_SEARCH_DIRS})

    IF (NOT _OVR_FOUND_INC_DIRS)
        request_ovr_search_directories()
    ENDIF (NOT _OVR_FOUND_INC_DIRS)
	  
	  FOREACH(_INC_DIR ${_OVR_FOUND_INC_DIRS})
        SET(OVR_INCLUDE_DIRS ${OVR_INCLUDE_DIRS} ${_INC_DIR} CACHE PATH "Oculus SDK include directory.")
    ENDFOREACH(_INC_DIR ${_OVR_FOUND_INC_DIRS})
    
ENDIF (NOT OVR_INCLUDE_DIRS)

IF(UNIX)
	SET(OVR_LIB_FILENAME "libovr.a")
ELSEIF(WIN32)
	SET(OVR_LIB_FILENAME "libovr64.lib")
ENDIF(UNIX)

IF ( NOT OVR_LIBRARY_DIRS )

    SET(_OVR_FOUND_LIB_DIR "")
    SET(_OVR_POSTFIX "")

    FOREACH(_SEARCH_DIR ${OVR_LIBRARY_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
				        NAMES ${OVR_LIB_FILENAME}
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _OVR_FOUND_LIB_DIR ${_SEARCH_DIR})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${OVR_LIBRARY_SEARCH_DIRS})

    IF (NOT _OVR_FOUND_LIB_DIR)
        request_ovr_search_directories()
    ELSE (NOT _OVR_FOUND_LIB_DIR)
		    SET(OVR_LIBRARY_DIRS ${_OVR_FOUND_LIB_DIR} CACHE PATH "The Oculus SDK library directory")
    ENDIF (NOT _OVR_FOUND_LIB_DIR)
    
    FOREACH(_LIB_DIR ${_OVR_FOUND_LIB_DIR})
        LIST(APPEND _OVR_LIBRARIES ${OVR_LIB_FILENAME})
    ENDFOREACH(_LIB_DIR ${_OVR_FOUND_INC_DIRS})

    IF (_OVR_FOUND_LIB_DIR)
        SET(OVR_LIBRARIES ${_OVR_LIBRARIES} CACHE FILEPATH "The Oculus SDK library filename.")
    ENDIF (_OVR_FOUND_LIB_DIR)
    
ENDIF ( NOT OVR_LIBRARY_DIRS )

##############################################################################
# verify
##############################################################################
IF ( NOT OVR_INCLUDE_DIRS OR NOT OVR_LIBRARY_DIRS )
    request_ovr_search_directories()
ELSE ( NOT OVR_INCLUDE_DIRS OR NOT OVR_LIBRARY_DIRS ) 
    UNSET(OVR_INCLUDE_SEARCH_DIR CACHE)
    UNSET(OVR_LIBRARY_SEARCH_DIR CACHE)
    MESSAGE(STATUS "--  found matching Oculus SDK version")
ENDIF ( NOT OVR_INCLUDE_DIRS OR NOT OVR_LIBRARY_DIRS )
