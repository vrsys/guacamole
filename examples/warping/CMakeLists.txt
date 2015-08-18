# determine source and header files
file(GLOB_RECURSE EXAMPLE_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.c *.cpp *.cc)

get_filename_component(_EXAMPLE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
SET(_EXE_NAME example-${_EXAMPLE_NAME})

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

LIST(APPEND LIB_PATHS ${CMAKE_SOURCE_DIR}/lib)

LINK_DIRECTORIES(${LIB_PATHS})

INCLUDE_DIRECTORIES( ${INCLUDE_PATHS}
                     ${GUACAMOLE_SOURCE_DIR}/include
                     ${AWESOMIUM_INCLUDE_DIR}
                     ${GUACAMOLE_PLOD_INCLUDE_DIR}
                     ${GUACAMOLE_GUI_INCLUDE_DIR}
                     ${GUACAMOLE_OCULUS_INCLUDE_DIR}
                     ${OVR_INCLUDE_DIR} 
)

ADD_EXECUTABLE( ${_EXE_NAME}
    ${EXAMPLE_SRC}
)


ADD_DEPENDENCIES(${_EXE_NAME} guacamole-oculus)
LIST(APPEND LIBS guacamole-oculus guacamole ${OVR_LIBRARY})

IF (${PLUGIN_guacamole-gui})
	ADD_DEPENDENCIES(${_EXE_NAME} guacamole-gui)
	LIST(APPEND LIBS guacamole-gui)
ENDIF (${PLUGIN_guacamole-gui})

IF (${PLUGIN_guacamole-plod})
	ADD_DEPENDENCIES(${_EXE_NAME} guacamole-plod)
	LIST(APPEND LIBS guacamole-plod)
ENDIF (${PLUGIN_guacamole-plod})

IF (MSVC)
  TARGET_LINK_LIBRARIES( ${_EXE_NAME} ${LIBS}
                         debug ${ASSIMP_LIBRARY_DEBUG} optimized ${ASSIMP_LIBRARY_RELEASE})
ELSEIF (UNIX)
  TARGET_LINK_LIBRARIES( ${_EXE_NAME} ${LIBS}
                         general ${ASSIMP_LIBRARIES}
                       )
ENDIF (MSVC)

# copy runtime libraries as a post-build process
IF (MSVC)
    ADD_CUSTOM_COMMAND ( TARGET ${_EXE_NAME} POST_BUILD 
      COMMAND robocopy \"${GLOBAL_EXT_DIR}/bin/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\" *.dll /R:0 /W:0 /NP 
            & robocopy \"${LIBRARY_OUTPUT_PATH}/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\" *.dll /R:0 /W:0 /NP 
            \n if %ERRORLEVEL% LEQ 7 (exit /b 0) else (exit /b 1)
    )
ENDIF (MSVC)
