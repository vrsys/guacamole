###############################################################################
# additional dependencies
###############################################################################
# include(findGPUCAST)

###############################################################################
# determine source files
###############################################################################
FILE(GLOB         GUA_NURBS_RENDERER_SRC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/renderer/*.cpp)
FILE(GLOB         GUA_NURBS_RENDERER_INC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/renderer/*.hpp)

FILE(GLOB         GUA_NURBS_NODE_SRC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/node/*.cpp)
FILE(GLOB         GUA_NURBS_NODE_INC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/node/*.hpp)

FILE(GLOB_RECURSE GUA_NURBS_DETAILS_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/renderer/nurbs_geometry/*.cpp)
FILE(GLOB_RECURSE GUA_NURBS_DETAILS_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/renderer/nurbs_geometry/*.hpp)

SOURCE_GROUP("renderer" FILES ${GUA_NURBS_RENDERER_SRC})
SOURCE_GROUP("renderer" FILES ${GUA_NURBS_RENDERER_INC})

SOURCE_GROUP("details" FILES ${GUA_NURBS_DETAILS_SRC})
SOURCE_GROUP("details" FILES ${GUA_NURBS_DETAILS_INC})

SOURCE_GROUP("node" FILES ${GUA_NURBS_NODE_SRC})
SOURCE_GROUP("node" FILES ${GUA_NURBS_NODE_INC})

SET(GUACAMOLE_NURBS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-nurbs includes." )

LINK_DIRECTORIES(${LIB_PATHS})

INCLUDE_DIRECTORIES( ${INCLUDE_PATHS}
                     ${GUACAMOLE_SOURCE_DIR}
                     ${GUACAMOLE_NURBS_INCLUDE_DIR}
)

ADD_LIBRARY( guacamole-nurbs SHARED
    ${GUA_NURBS_NODE_SRC}
    ${GUA_NURBS_NODE_INC}
    ${GUA_NURBS_RENDERER_SRC}
    ${GUA_NURBS_RENDERER_INC}
    ${GUA_NURBS_DETAILS_SRC}
    ${GUA_NURBS_DETAILS_INC}
)

ADD_DEPENDENCIES(guacamole-nurbs guacamole)

###############################################################################
# compiling and linking
###############################################################################
IF (MSVC)
	set_target_properties(guacamole-nurbs PROPERTIES COMPILE_FLAGS "-D GUA_NURBS_LIBRARY")
ENDIF (MSVC)

IF (UNIX)
    LIST(APPEND LIBS udev Xinerama guacamole)
ELSEIF (MSVC)
    LIST(APPEND LIBS winmm.lib guacamole)
ENDIF(UNIX)

target_link_libraries( guacamole-nurbs guacamole)
