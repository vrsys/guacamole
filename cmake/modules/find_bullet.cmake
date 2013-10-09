SET(BULLET_INCLUDE_SEARCH_DIRS
    /opt/bullet/current
)

SET(BULLET_LIBRARY_SEARCH_DIRS
    /opt/bullet/current/bullet-build
)

message("-- checking for BULLET")

IF (NOT BULLET_INCLUDE_DIRS)

    SET(_BULLET_FOUND_INC_DIRS "")

    FOREACH(_SEARCH_DIR ${BULLET_INCLUDE_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
            NAMES src/btBulletDynamicsCommon.h
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _BULLET_FOUND_INC_DIRS ${_CUR_SEARCH})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${BULLET_INCLUDE_SEARCH_DIRS})

    IF (NOT _BULLET_FOUND_INC_DIRS)
        MESSAGE(FATAL_ERROR "find_bullet.cmake: unable to find bullet headers")
    ENDIF (NOT _BULLET_FOUND_INC_DIRS)
    
    FOREACH(_INC_DIR ${_BULLET_FOUND_INC_DIRS})
        LIST(APPEND BULLET_INCLUDE_DIRS ${_INC_DIR}/src)
        LIST(APPEND BULLET_INCLUDE_DIRS ${_INC_DIR}/Extras/HACD)
    ENDFOREACH(_INC_DIR ${_BOOST_FOUND_INC_DIRS})

ENDIF(NOT BULLET_INCLUDE_DIRS)

IF (        BULLET_INCLUDE_DIRS
    AND NOT BULLET_LIBRARIES)

    SET(_BULLET_FOUND_LIB_DIR "")
    SET(_BULLET_POSTFIX "")

    FOREACH(_SEARCH_DIR ${BULLET_LIBRARY_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
            NAMES src/BulletDynamics/libBulletDynamics.so
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _BULLET_FOUND_LIB_DIR ${_SEARCH_DIR})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${BULLET_LIBRARY_SEARCH_DIRS})

    IF (NOT _BULLET_FOUND_LIB_DIR)
        MESSAGE(FATAL_ERROR "find_bullet.cmake: unable to find bullet libraries")
    ELSE (NOT _BULLET_FOUND_LIB_DIR)
        message("--  found matching version")
    ENDIF (NOT _BULLET_FOUND_LIB_DIR)
    
    FOREACH(_LIB_DIR ${_BULLET_FOUND_LIB_DIR})
        LIST(APPEND BULLET_LIBRARIES ${_LIB_DIR}/src/BulletCollision/libBulletCollision.so)
        LIST(APPEND BULLET_LIBRARIES ${_LIB_DIR}/src/BulletDynamics/libBulletDynamics.so)
        LIST(APPEND BULLET_LIBRARIES ${_LIB_DIR}/src/LinearMath/libLinearMath.so)
        LIST(APPEND BULLET_LIBRARIES ${_LIB_DIR}/Extras/HACD/libHACD.so)
    ENDFOREACH(_LIB_DIR ${_BULLET_FOUND_INC_DIRS})
    

ENDIF(        BULLET_INCLUDE_DIRS
      AND NOT BULLET_LIBRARIES)



