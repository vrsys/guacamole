# determine source and header files
file(GLOB_RECURSE GUACAMOLE_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    gua/*.c
    gua/*.cpp
    ../include/gua/*.h
    ../include/gua/*.hpp
)

LINK_DIRECTORIES(${LIB_PATHS})

IF (UNIX)
ADD_LIBRARY( guacamole SHARED
    ${GUACAMOLE_SRC}
)
ELSEIF (WIN32)
ADD_LIBRARY( guacamole STATIC
    ${GUACAMOLE_SRC}
)
ENDIF(UNIX)

INCLUDE_DIRECTORIES(guacamole ${INCLUDE_PATHS})

TARGET_LINK_LIBRARIES( guacamole debug ${LIBS} optimized ${LIBS})

ADD_DEPENDENCIES(guacamole guarc)
