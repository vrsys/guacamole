###############################################################################

MACRO(GET_SUBDIRECTORIES subdirectories current_directory)

  FILE(GLOB directory_content RELATIVE ${current_directory} *)
  SET(dirlist "")
  
  FOREACH(child ${directory_content})
  
    IF(IS_DIRECTORY ${current_directory}/${child})
        SET(dirlist ${dirlist} ${child})
    ENDIF()
	
  ENDFOREACH()
  
  SET(${subdirectories} ${dirlist})
  
ENDMACRO()

###############################################################################

# copy_runtime_dependencies [target_name] [path_to_runtime_libraries] [executable_path]
MACRO(COPY_RUNTIME_DEPENDENCIES _TARGET_NAME _RUNTIME_LIBRARY_PATH _TARGET_PATH)

	SET ( _COPY_COMMAND "robocopy" )
	SET ( _PUSH_DIRECTORY_COMMAND "pushd" )
	SET ( _POP_DIRECTORY_COMMAND "popd" )
	
	IF (WIN32)

    MAKE_DIRECTORY(${_TARGET_PATH})

    IF ("${_RUNTIME_LIBARY_PATH}" STREQUAL "")
      SET(_RUNTIME_LIBARY_PATH .)
    ENDIF ("${_RUNTIME_LIBARY_PATH}" STREQUAL "")

		SET(_POST_PROCESS_COMMAND ${_COPY_COMMAND} "${_RUNTIME_LIBARY_PATH}" "${_TARGET_PATH}" *.dll /R:0 /W:0 > nul)

    #SET(_POST_PROCESS_COMMAND 
			  #${_PUSH_DIRECTORY_COMMAND} ${_RUNTIME_LIBARY_PATH} \n
			  #${_COPY_COMMAND} /Y *.dll "${_TARGET_PATH}" \n 
			  #${_POP_DIRECTORY_COMMAND}
    #)
		ADD_CUSTOM_COMMAND ( TARGET ${_TARGET_NAME} POST_BUILD COMMAND ${_POST_PROCESS_COMMAND})

	ENDIF(WIN32)
	
ENDMACRO(COPY_RUNTIME_DEPENDENCIES)
