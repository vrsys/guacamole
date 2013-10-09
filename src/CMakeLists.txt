# determine source and header files
file(GLOB_RECURSE GUACAMOLE_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    gua/*.c
    gua/*.cpp
    ../include/gua/*.h
    ../include/gua/*.hpp
)

ADD_LIBRARY( guacamole SHARED
    ${GUACAMOLE_SRC}
)

TARGET_LINK_LIBRARIES( guacamole  ${LIBS} )
ADD_DEPENDENCIES(guacamole guarc)
