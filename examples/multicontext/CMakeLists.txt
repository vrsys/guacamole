get_filename_component(_EXAMPLE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
SET(_EXE_NAME example-${_EXAMPLE_NAME})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

list(APPEND LIB_PATHS ${CMAKE_SOURCE_DIR}/lib)

link_directories(${LIB_PATHS})

include_directories( ${INCLUDE_PATHS} 
                     ${PROJECT_SOURCE_DIR}/include
                     ${GUACAMOLE_NURBS_INCLUDE_DIR}
                    )

add_executable( ${_EXE_NAME} main.cpp)

add_dependencies(${_EXE_NAME} guacamole guacamole-nurbs)

list(APPEND LIBS guacamole)

target_link_libraries( ${_EXE_NAME} ${LIBS} guacamole-nurbs)

# copy runtime libraries as a post-build process
if (MSVC)

    COPY_RUNTIME_DEPENDENCIES ( ${_EXE_NAME} \"${GLOBAL_EXT_DIR}/bin/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\")
    COPY_RUNTIME_DEPENDENCIES ( ${_EXE_NAME} \"${LIBRARY_OUTPUT_PATH}/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\")
    COPY_RUNTIME_DEPENDENCIES ( ${_EXE_NAME} \"${GPUCAST_RUNTIME_LIBRARY_DIR}/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\")

    FOREACH (_LIB_DIR ${GUACAMOLE_LIBRARY_DIRS})
        COPY_RUNTIME_DEPENDENCIES ( ${_EXE_NAME} \"${_LIB_DIR}/$(Configuration)/\" \"${EXECUTABLE_OUTPUT_PATH}/$(Configuration)/\")
    ENDFOREACH (_LIB_DIR ${GUACAMOLE_LIBRARY_DIRS})

endif (MSVC)
