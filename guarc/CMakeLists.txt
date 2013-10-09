# determine source and header files
file(GLOB_RECURSE sources RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.c *.cpp *.h *.hpp)

SET(_EXE_NAME guarc)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

ADD_EXECUTABLE( ${_EXE_NAME}
    ${sources}
)
