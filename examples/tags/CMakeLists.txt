# determine source and header files

get_filename_component(_EXE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

add_executable( ${_EXE_NAME} main.cpp)

target_link_libraries( ${_EXE_NAME} guacamole)

# copy runtime libraries as a post-build process
if (MSVC)

    COPY_RUNTIME_DEPENDENCIES ( ${_EXE_NAME} \"${GLOBAL_EXT_DIR}/bin/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\")
    COPY_RUNTIME_DEPENDENCIES ( ${_EXE_NAME} \"${LIBRARY_OUTPUT_PATH}/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\")

    FOREACH (_LIB_DIR ${GUACAMOLE_LIBRARY_DIRS})
        COPY_RUNTIME_DEPENDENCIES ( ${_EXE_NAME} \"${_LIB_DIR}/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\")
    ENDFOREACH (_LIB_DIR ${GUACAMOLE_LIBRARY_DIRS})

endif (MSVC)
