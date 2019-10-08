# determine source and header files
file(GLOB_RECURSE EXAMPLE_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.c *.cpp *.cc)

get_filename_component(_EXAMPLE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
SET(_EXE_NAME example-${_EXAMPLE_NAME})

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

INCLUDE_DIRECTORIES( ${INCLUDE_PATHS} 
                     ${GUACAMOLE_SOURCE_DIR}/include
                     ${OVR_INCLUDE_DIR} 
                     ${GUACAMOLE_OCULUS_INCLUDE_DIR}
                     ${GUACAMOLE_LOD_INCLUDE_DIR}
)

ADD_EXECUTABLE( ${_EXE_NAME}
    ${EXAMPLE_SRC}
)

ADD_DEPENDENCIES(${_EXE_NAME} guacamole guacamole-PLOD guacamole-oculus)

LIST(APPEND LIBS guacamole-oculus guacamole ${OVR_LIBRARIES})

IF (MSVC)
  TARGET_LINK_LIBRARIES( ${_EXE_NAME} ${LIBS}
                         debug ${ASSIMP_LIBRARY_DEBUG} optimized ${ASSIMP_LIBRARY_RELEASE})
ELSEIF (UNIX)
  TARGET_LINK_LIBRARIES( ${_EXE_NAME} ${LIBS} 
			 general ${OVR_LIBRARY}
                         general ${ASSIMP_LIBRARIES}
                       )
ENDIF (MSVC)

target_link_libraries( ${_EXE_NAME} ${LIBS} guacamole-LOD guacamole-oculus)

# copy runtime libraries as a post-build process
IF (MSVC)
  FOREACH(_LIB ${GUACAMOLE_RUNTIME_LIBRARIES})
    get_filename_component(_FILE ${_LIB} NAME)
    get_filename_component(_PATH ${_LIB} DIRECTORY)
    SET(COPY_DLL_COMMAND_STRING ${COPY_DLL_COMMAND_STRING} robocopy \"${_PATH}\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\" ${_FILE} /R:0 /W:0 /NP > nul &)
  ENDFOREACH()

  SET(COPY_DLL_COMMAND_STRING ${COPY_DLL_COMMAND_STRING} robocopy \"${LIBRARY_OUTPUT_PATH}/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\" *.dll /R:0 /W:0 /NP > nul &)
  ADD_CUSTOM_COMMAND ( TARGET ${_EXE_NAME} POST_BUILD COMMAND ${COPY_DLL_COMMAND_STRING} \n if %ERRORLEVEL% LEQ 7 (exit /b 0) else (exit /b 1))
ENDIF (MSVC)
