SET(SCHISM_INCLUDE_SEARCH_DIRS
    #/opt/schism/schism_debug
    /opt/schism/current
    #/opt/schism/schism_debug
)

SET(SCHISM_LIBRARY_SEARCH_DIRS
    #/opt/schism/schism_debug/lib/linux_x86
    /opt/schism/current/lib/linux_x86
    #/opt/schism/schism_debug/lib/linux_x86
)

message("-- checking for schism")

IF (NOT SCHISM_INCLUDE_DIRS)

    SET(_SCHISM_FOUND_INC_DIRS "")

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
        MESSAGE(FATAL_ERROR "find_schism.cmake: unable to find SCHISM headers")
    ENDIF (NOT _SCHISM_FOUND_INC_DIRS)
    
    FOREACH(_INC_DIR ${_SCHISM_FOUND_INC_DIRS})
        LIST(APPEND SCHISM_INCLUDE_DIRS ${_INC_DIR}/scm_cl_core/src)
        LIST(APPEND SCHISM_INCLUDE_DIRS ${_INC_DIR}/scm_core/src)
        LIST(APPEND SCHISM_INCLUDE_DIRS ${_INC_DIR}/scm_gl_core/src)
        LIST(APPEND SCHISM_INCLUDE_DIRS ${_INC_DIR}/scm_gl_util/src)
        LIST(APPEND SCHISM_INCLUDE_DIRS ${_INC_DIR}/scm_input/src)
    ENDFOREACH(_INC_DIR ${_BOOST_FOUND_INC_DIRS})

ENDIF(NOT SCHISM_INCLUDE_DIRS)

IF (        SCHISM_INCLUDE_DIRS
    AND NOT SCHISM_LIBRARIES)

    SET(_SCHISM_FOUND_LIB_DIR "")
    SET(_SCHISM_POSTFIX "")

    FOREACH(_SEARCH_DIR ${SCHISM_LIBRARY_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
                NAMES libscm_gl_core.so
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _SCHISM_FOUND_LIB_DIR ${_SEARCH_DIR})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${SCHISM_LIBRARY_SEARCH_DIRS})

    IF (NOT _SCHISM_FOUND_LIB_DIR)
        MESSAGE(FATAL_ERROR "find_schism.cmake: unable to find SCHISM library")
    ELSE (NOT _SCHISM_FOUND_LIB_DIR)
        message("--  found matching version")
    ENDIF (NOT _SCHISM_FOUND_LIB_DIR)
    
    FOREACH(_LIB_DIR ${_SCHISM_FOUND_LIB_DIR})
        file(GLOB SCHISM_LIBRARIES ${_LIB_DIR}/*.so)
    ENDFOREACH(_LIB_DIR ${_SCHISM_FOUND_INC_DIRS})
    

ENDIF(        SCHISM_INCLUDE_DIRS
      AND NOT SCHISM_LIBRARIES)



