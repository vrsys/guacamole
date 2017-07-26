###############################################################################
# additional dependencies
###############################################################################
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(FindGPUcast)

###############################################################################
# determine source files
###############################################################################
FILE(GLOB         GUA_NURBS_RENDERER_SRC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/renderer/*.cpp)
FILE(GLOB         GUA_NURBS_RENDERER_INC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/renderer/*.hpp)

FILE(GLOB         GUA_NURBS_NODE_SRC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/node/*.cpp)
FILE(GLOB         GUA_NURBS_NODE_INC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/node/*.hpp)

FILE(GLOB_RECURSE GUA_NURBS_DETAILS_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/renderer/detail/*.cpp)
FILE(GLOB_RECURSE GUA_NURBS_DETAILS_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/renderer/detail/*.hpp)

FILE(GLOB_RECURSE GUA_NURBS_SHADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*)

SOURCE_GROUP("renderer" FILES ${GUA_NURBS_RENDERER_SRC})
SOURCE_GROUP("renderer" FILES ${GUA_NURBS_RENDERER_INC})

SOURCE_GROUP("details" FILES ${GUA_NURBS_DETAILS_SRC})
SOURCE_GROUP("details" FILES ${GUA_NURBS_DETAILS_INC})

SOURCE_GROUP("node" FILES ${GUA_NURBS_NODE_SRC})
SOURCE_GROUP("node" FILES ${GUA_NURBS_NODE_INC})

SOURCE_GROUP("resources" FILES ${GUA_NURBS_SHADERS})

SET(GUACAMOLE_NURBS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-nurbs includes." )

add_library( guacamole-nurbs SHARED
    ${GUA_NURBS_NODE_SRC}
    ${GUA_NURBS_NODE_INC}
    ${GUA_NURBS_RENDERER_SRC}
    ${GUA_NURBS_RENDERER_INC}
    ${GUA_NURBS_DETAILS_SRC}
    ${GUA_NURBS_DETAILS_INC}
    ${GUA_NURBS_SHADERS}
)

target_include_directories( guacamole-nurbs PUBLIC
                     ${GUACAMOLE_NURBS_INCLUDE_DIR}
                     ${GPUCAST_INCLUDE_DIR}
)

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

IF (MSVC)
  target_link_libraries( guacamole-nurbs guacamole optimized ${GPUCAST_LIBRARY_RELEASE})
  target_link_libraries( guacamole-nurbs guacamole debug ${GPUCAST_LIBRARY_DEBUG})
ELSEIF (UNIX)
  target_link_libraries( guacamole-nurbs guacamole ${GPUCAST_LIBRARY})
ENDIF (MSVC)

###############################################################################
# install 
###############################################################################

IF (MSVC)
  install (TARGETS guacamole-nurbs
    CONFIGURATIONS Release
    RUNTIME DESTINATION bin/Release
    LIBRARY DESTINATION lib/Release
    ARCHIVE DESTINATION lib/Release
  )

  install (TARGETS guacamole-nurbs
    CONFIGURATIONS Debug
    RUNTIME DESTINATION bin/Debug
    LIBRARY DESTINATION lib/Debug
    ARCHIVE DESTINATION lib/Debug
  )
ELSEIF (UNIX)
  install (TARGETS guacamole-nurbs 
    RUNTIME DESTINATION lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
ENDIF (MSVC)

# header files 
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/ DESTINATION include/gua FILES_MATCHING PATTERN "*.hpp")

# shader and material resources
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources/ DESTINATION resources 
        FILES_MATCHING 
        PATTERN "*.frag" 
        PATTERN "*.vert" 
        PATTERN "*.glsl"
        PATTERN "*.geom"
        PATTERN "*.tctrl"
        PATTERN "*.teval"
        PATTERN "*.obj"
        PATTERN "*.gmd"
        )
