##############################################################################
# pre-defined search paths
##############################################################################
SET(CUDA_INCLUDE_SEARCH_DIRS
	${GLOBAL_EXT_DIR}/inc/cuda
	/opt/cuda/current/cuda/include
    /usr/include
	/usr/local/include
)

SET(CUDA_LIBRARY_SEARCH_DIRS
	${GLOBAL_EXT_DIR}/lib
    /opt/cuda/current/cuda/lib
	/usr/lib
	/usr/local/lib
)

##############################################################################
# feedback to provide user-defined paths to search for cuda
##############################################################################
MACRO (request_cuda_search_directories)
    
    IF ( NOT CUDA_INCLUDE_DIRS AND NOT CUDA_LIBRARY_DIRS )
        SET(CUDA_INCLUDE_SEARCH_DIR "Please provide cuda include path." CACHE PATH "path to cuda headers.")
        SET(CUDA_LIBRARY_SEARCH_DIR "Please provide cuda library path." CACHE PATH "path to cuda libraries.")
        MESSAGE(FATAL_ERROR "find_cuda.cmake: unable to find cuda.")
    ENDIF ( NOT CUDA_INCLUDE_DIRS AND NOT CUDA_LIBRARY_DIRS )

    IF ( NOT CUDA_INCLUDE_DIRS )
        SET(CUDA_INCLUDE_SEARCH_DIR "Please provide cuda include path." CACHE PATH "path to cuda headers.")
        MESSAGE(FATAL_ERROR "find_cuda.cmake: unable to find cuda headers.")
    ELSE ( NOT CUDA_INCLUDE_DIRS )
        UNSET(CUDA_INCLUDE_SEARCH_DIR CACHE)
    ENDIF ( NOT CUDA_INCLUDE_DIRS )

    IF ( NOT CUDA_LIBRARY_DIRS )
        SET(CUDA_LIBRARY_SEARCH_DIR "Please provide cuda library path." CACHE PATH "path to cuda libraries.")
        MESSAGE(FATAL_ERROR "find_cuda.cmake: unable to find cuda libraries.")
    ELSE ( NOT CUDA_LIBRARY_DIRS )
        UNSET(CUDA_LIBRARY_SEARCH_DIR CACHE)
    ENDIF ( NOT CUDA_LIBRARY_DIRS ) 

ENDMACRO (request_cuda_search_directories)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for CUDA")

IF (NOT CUDA_INCLUDE_DIRS)

    SET(_CUDA_FOUND_INC_DIRS "")

    FOREACH(_SEARCH_DIR ${CUDA_INCLUDE_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
            NAMES cuda.h
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _CUDA_FOUND_INC_DIRS ${_CUR_SEARCH})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${CUDA_INCLUDE_SEARCH_DIRS})

    IF (NOT _CUDA_FOUND_INC_DIRS)
        request_cuda_search_directories()
    ENDIF (NOT _CUDA_FOUND_INC_DIRS)
	
	  FOREACH(_INC_DIR ${_CUDA_FOUND_INC_DIRS})
        LIST(APPEND _CUDA_INCLUDE_DIRS ${_INC_DIR})
    ENDFOREACH(_INC_DIR ${_BOOST_FOUND_INC_DIRS})
    
    IF (_CUDA_FOUND_INC_DIRS)
        SET(CUDA_INCLUDE_DIRS ${_CUDA_INCLUDE_DIRS} CACHE PATH "path to cuda headers.")
    ENDIF (_CUDA_FOUND_INC_DIRS)

ENDIF(NOT CUDA_INCLUDE_DIRS)

IF(UNIX)
	LIST(APPEND _CUDA_LIB_FILENAMES "libcudart.so")
ELSEIF(WIN32)
    LIST(APPEND _CUDA_LIB_FILENAMES "cudart.lib")
	LIST(APPEND _CUDA_LIB_FILENAMES "cuda.lib")
ENDIF(UNIX)

IF ( CUDA_INCLUDE_DIRS AND NOT CUDA_LIBRARIES )

    SET(_CUDA_FOUND_LIB_DIR "")
    SET(_CUDA_POSTFIX "")
	LIST(GET _CUDA_LIB_FILENAMES 0 _CUDA_RUNTIME_LIBRARY)

    FOREACH(_SEARCH_DIR ${CUDA_LIBRARY_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
				NAMES ${_CUDA_RUNTIME_LIBRARY}
                PATHS ${_SEARCH_DIR}
				PATH_SUFFIXES debug release
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _CUDA_FOUND_LIB_DIR ${_SEARCH_DIR})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${CUDA_LIBRARY_SEARCH_DIRS})

    IF (NOT _CUDA_FOUND_LIB_DIR)
      request_cuda_search_directories()
    ELSE (NOT _CUDA_FOUND_LIB_DIR)
        SET(CUDA_LIBRARY_DIRS ${_CUDA_FOUND_LIB_DIR} CACHE STRING "The cuda library directory.")
        message(STATUS "--  found matching version")
    ENDIF (NOT _CUDA_FOUND_LIB_DIR)
    
    IF (_CUDA_FOUND_LIB_DIR)
        SET(CUDA_LIBRARIES ${_CUDA_LIB_FILENAMES} CACHE STRING "The cuda libraries.")
    ENDIF (_CUDA_FOUND_LIB_DIR)
    
ENDIF( CUDA_INCLUDE_DIRS AND NOT CUDA_LIBRARIES )

##############################################################################
# verify
##############################################################################
IF ( NOT CUDA_INCLUDE_DIRS OR NOT CUDA_LIBRARY_DIRS )
    request_cuda_search_directories()
ELSE ( NOT CUDA_INCLUDE_DIRS OR NOT CUDA_LIBRARY_DIRS )
    UNSET(CUDA_INCLUDE_SEARCH_DIR CACHE)
    UNSET(CUDA_LIBRARY_SEARCH_DIR CACHE)
    MESSAGE(STATUS "--  found matching cuda version")
ENDIF ( NOT CUDA_INCLUDE_DIRS OR NOT CUDA_LIBRARY_DIRS )