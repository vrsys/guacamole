# determine source and header files
file(GLOB_RECURSE EXAMPLE_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.c *.cpp *.cc)

get_filename_component(_EXAMPLE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
SET(_EXE_NAME example-${_EXAMPLE_NAME})

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

LIST(APPEND LIB_PATHS ${CMAKE_SOURCE_DIR}/lib)

LINK_DIRECTORIES(${LIB_PATHS})

INCLUDE_DIRECTORIES( ${INCLUDE_PATHS} 
                     ${GUACAMOLE_SOURCE_DIR}/include
                     ${OVR_INCLUDE_DIR} 
                     ${GUACAMOLE_OCULUS_INCLUDE_DIR}
)

ADD_EXECUTABLE( ${_EXE_NAME}
    ${EXAMPLE_SRC}
)

ADD_DEPENDENCIES(${_EXE_NAME} guacamole-oculus)

LIST(APPEND LIBS guacamole-oculus guacamole)

IF (MSVC)
  TARGET_LINK_LIBRARIES( ${_EXE_NAME} ${LIBS}
                         debug ${ASSIMP_LIBRARY_DEBUG} optimized ${ASSIMP_LIBRARY_RELEASE})
ELSEIF (UNIX)
  TARGET_LINK_LIBRARIES( ${_EXE_NAME} ${LIBS} 
			 general ${OVR_LIBRARY}
                         general ${ASSIMP_LIBRARIES}
                       )
ENDIF (MSVC)

# copy runtime libraries as a post-build process
IF (MSVC)

    COPY_RUNTIME_DEPENDENCIES ( ${_EXE_NAME} \"${GLOBAL_EXT_DIR}/bin/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\")
    COPY_RUNTIME_DEPENDENCIES ( ${_EXE_NAME} \"${LIBRARY_OUTPUT_PATH}/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\")

    FOREACH (_LIB_DIR ${GUACAMOLE_LIBRARY_DIRS})
        COPY_RUNTIME_DEPENDENCIES ( ${_EXE_NAME} \"${_LIB_DIR}/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\")
    ENDFOREACH (_LIB_DIR ${GUACAMOLE_LIBRARY_DIRS})
ENDIF (MSVC)
