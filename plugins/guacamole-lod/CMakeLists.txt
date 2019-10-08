###############################################################################
# additional dependencies
###############################################################################
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(LAMURE_ROOT "/opt/lamure/install" CACHE PATH "Set to your lamure install path.")
include(find_lamure)

###############################################################################
# determine source files
###############################################################################
FILE(GLOB         GUA_LOD_RENDERER_SRC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/renderer/*.cpp)
FILE(GLOB         GUA_LOD_RENDERER_INC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/renderer/*.hpp)

FILE(GLOB         GUA_LOD_NODE_SRC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/node/*.cpp)
FILE(GLOB         GUA_LOD_NODE_INC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/node/*.hpp)

FILE(GLOB_RECURSE GUA_LOD_SHADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*)

SOURCE_GROUP("renderer" FILES ${GUA_LOD_RENDERER_SRC})
SOURCE_GROUP("renderer" FILES ${GUA_LOD_RENDERER_INC})

SOURCE_GROUP("node" FILES ${GUA_LOD_NODE_SRC})
SOURCE_GROUP("node" FILES ${GUA_LOD_NODE_INC})

SOURCE_GROUP("resources" FILES ${GUA_LOD_SHADERS})

SET(GUACAMOLE_LOD_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-lod includes." )

add_library( guacamole-lod SHARED
    ${GUA_LOD_NODE_SRC}
    ${GUA_LOD_NODE_INC}
    ${GUA_LOD_RENDERER_SRC}
    ${GUA_LOD_RENDERER_INC}
    ${GUA_LOD_DETAILS_SRC}
    ${GUA_LOD_DETAILS_INC}
    ${GUA_LOD_SHADERS}
)

target_include_directories(guacamole-lod PUBLIC
                     ${GUACAMOLE_LOD_INCLUDE_DIR}
                     ${LAMURE_INCLUDE_DIRS}
)
#find_package(LOD REQUIRED)

###############################################################################
# compiling and linking
###############################################################################
IF (MSVC)
	set_target_properties(guacamole-lod PROPERTIES COMPILE_FLAGS "-D GUA_LOD_LIBRARY")
ENDIF (MSVC)

IF (UNIX)
    LIST(APPEND LIBS udev Xinerama guacamole)
ELSEIF (MSVC)
    LIST(APPEND LIBS winmm.lib guacamole)
ENDIF(UNIX)

IF (MSVC)
  target_link_libraries( guacamole-lod optimized ${LAMURE_RENDERING_LIBRARY}

                                        optimized ${LAMURE_COMMON_LIBRARY}
                                        #debug ${LAMURE_RENDERING_LIBRARY_DEBUG}
                                        #debug ${LAMURE_COMMON_LIBRARY_DEBUG}
                                        general guacamole
                       )
ELSEIF (UNIX)
  target_link_libraries( guacamole-lod 
                         guacamole 
                         ${LAMURE_RENDERING_LIBRARY}
                         ${LAMURE_COMMON_LIBRARY}
                         ${LAMURE_PROVENANCE_LIBRARY}
                       )

ENDIF (MSVC)

###############################################################################
# install 
###############################################################################

IF (MSVC)
  install (TARGETS guacamole-lod
    CONFIGURATIONS Release
    RUNTIME DESTINATION bin/Release
    LIBRARY DESTINATION lib/Release
    ARCHIVE DESTINATION lib/Release
  )

  install (TARGETS guacamole-lod
    CONFIGURATIONS Debug
    RUNTIME DESTINATION bin/Debug
    LIBRARY DESTINATION lib/Debug
    ARCHIVE DESTINATION lib/Debug
  )
ELSEIF (UNIX)
  install (TARGETS guacamole-lod 
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
        )
