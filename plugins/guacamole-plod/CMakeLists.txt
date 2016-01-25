###############################################################################
# additional dependencies
###############################################################################
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(PBR_ROOT "/opt/pbr/inst_cb" CACHE PATH "Set to your PLOD install path.")
include(FindPBR)

###############################################################################
# determine source files
###############################################################################
FILE(GLOB         GUA_PLOD_RENDERER_SRC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/renderer/*.cpp)
FILE(GLOB         GUA_PLOD_RENDERER_INC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/renderer/*.hpp)

FILE(GLOB         GUA_PLOD_NODE_SRC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/node/*.cpp)
FILE(GLOB         GUA_PLOD_NODE_INC       RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/node/*.hpp)

FILE(GLOB_RECURSE GUA_PLOD_SHADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*)

SOURCE_GROUP("renderer" FILES ${GUA_PLOD_RENDERER_SRC})
SOURCE_GROUP("renderer" FILES ${GUA_PLOD_RENDERER_INC})

SOURCE_GROUP("node" FILES ${GUA_PLOD_NODE_SRC})
SOURCE_GROUP("node" FILES ${GUA_PLOD_NODE_INC})

SOURCE_GROUP("resources" FILES ${GUA_PLOD_SHADERS})

SET(GUACAMOLE_PLOD_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-plod includes." )

LINK_DIRECTORIES(${LIB_PATHS})

INCLUDE_DIRECTORIES( ${INCLUDE_PATHS}
                     ${GUACAMOLE_SOURCE_DIR}
                     ${GUACAMOLE_PLOD_INCLUDE_DIR}
                     ${PBR_INCLUDE_DIRS}
)

ADD_LIBRARY( guacamole-plod STATIC
    ${GUA_PLOD_NODE_SRC}
    ${GUA_PLOD_NODE_INC}
    ${GUA_PLOD_RENDERER_SRC}
    ${GUA_PLOD_RENDERER_INC}
    ${GUA_PLOD_DETAILS_SRC}
    ${GUA_PLOD_DETAILS_INC}
    ${GUA_PLOD_SHADERS}
)

ADD_DEPENDENCIES(guacamole-plod guacamole)

LIST(APPEND LIB_PATHS ${PROTOBUF_LIBRARY_DIRS})
LIST(APPEND INCLUDE_PATHS ${PROTOBUF_INCLUDE_DIRS})

include(find_protobuf)
#set(PLOD_ROOT "/opt/pbr/gualod/trunk/pbr" CACHE PATH "Set to your PLOD install path.")

#find_package(PLOD REQUIRED)

message(STATUS " protobuf:" )
message(STATUS "   library: ${PROTOBUF_LIBRARIES}" )
message(STATUS "   library path: ${PROTOBUF_LIBRARY_DIRS}" )
message(STATUS "   include: ${PROTOBUF_INCLUDE_DIRS}" )
message(STATUS "" )

###############################################################################
# compiling and linking
###############################################################################
IF (MSVC)
  set_target_properties(guacamole-plod PROPERTIES COMPILE_FLAGS "-D GUA_PLOD_LIBRARY")
ENDIF (MSVC)

IF (UNIX)
    LIST(APPEND LIBS udev Xinerama guacamole)
ELSEIF (MSVC)
    LIST(APPEND LIBS winmm.lib guacamole)
ENDIF(UNIX)

IF (MSVC)
  target_link_libraries( guacamole-plod optimized ${PBR_RENDERING_LIBRARY}
                                        optimized ${PBR_COMMON_LIBRARY}
                                        debug ${PBR_RENDERING_LIBRARY_DEBUG}
                                        debug ${PBR_COMMON_LIBRARY_DEBUG} 
                                        general guacamole
                       )
ELSEIF (UNIX)
  target_link_libraries( guacamole-plod 
                         guacamole 
                         ${PBR_RENDERING_LIBRARY}
                         ${PBR_COMMON_LIBRARY}
                         ${PROTOBUF_LIBRARIES}
                       )

ENDIF (MSVC)

###############################################################################
# install 
###############################################################################

IF (MSVC)
  install (TARGETS guacamole-plod
    CONFIGURATIONS Release
    RUNTIME DESTINATION bin/Release
    LIBRARY DESTINATION lib/Release
    ARCHIVE DESTINATION lib/Release
  )

  install (TARGETS guacamole-plod
    CONFIGURATIONS Debug
    RUNTIME DESTINATION bin/Debug
    LIBRARY DESTINATION lib/Debug
    ARCHIVE DESTINATION lib/Debug
  )
ELSEIF (UNIX)
  install (TARGETS guacamole-plod 
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
