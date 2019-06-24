# determine source and header files
file(GLOB_RECURSE EXAMPLE_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.c *.cpp *.cc)

get_filename_component(_EXAMPLE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
SET(_EXE_NAME example-${_EXAMPLE_NAME})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(${_EXE_NAME} ${EXAMPLE_SRC})

target_link_libraries(${_EXE_NAME} guacamole-gui guacamole-PLOD)

# copy runtime libraries as a post-build process
IF (MSVC)
    ADD_CUSTOM_COMMAND ( TARGET ${_EXE_NAME} POST_BUILD 
      COMMAND robocopy \"${GLOBAL_EXT_DIR}/bin/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\" *.dll /R:0 /W:0 /NP /njh /njs /ndl /nc /ns
            & robocopy \"${LIBRARY_OUTPUT_PATH}/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\" *.dll /R:0 /W:0 /NP /njh /njs /ndl /nc /ns
            \n if %ERRORLEVEL% LEQ 7 (exit /b 0) else (exit /b 1)
    )
ENDIF (MSVC)
