# dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# determine source and header files
file(GLOB GUA_VIDEO3D_INC RELATIVE                ${CMAKE_CURRENT_SOURCE_DIR} include/gua/video3d/*.hpp)
file(GLOB GUA_VIDEO3D_SRC RELATIVE                ${CMAKE_CURRENT_SOURCE_DIR} src/gua/video3d/*.cpp)
file(GLOB_RECURSE GUA_VIDEO3D_DETAIL_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/video3d/video3d_geometry/*.hpp include/gua/video3d/video3d_geometry/*.h)
file(GLOB_RECURSE GUA_VIDEO3D_DETAIL_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/video3d/video3d_geometry/*.cpp)
FILE(GLOB_RECURSE GUA_VIDEO3D_SHADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*)

SOURCE_GROUP("renderer" FILES ${GUA_VIDEO3D_INC})
SOURCE_GROUP("renderer" FILES ${GUA_VIDEO3D_SRC})
SOURCE_GROUP("detail"   FILES ${GUA_VIDEO3D_DETAIL_INC})
SOURCE_GROUP("detail"   FILES ${GUA_VIDEO3D_DETAIL_SRC})
SOURCE_GROUP("resources" FILES ${GUA_VIDEO3D_SHADERS})

SET(GUACAMOLE_VIDEO3D_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-video3d includes.")

add_library( guacamole-video3d SHARED
    ${GUA_VIDEO3D_INC}
    ${GUA_VIDEO3D_SRC}
    ${GUA_VIDEO3D_DETAIL_INC}
    ${GUA_VIDEO3D_DETAIL_SRC}
    ${GUA_VIDEO3D_SHADERS}
)

target_include_directories(guacamole-video3d PUBLIC ${GUACAMOLE_VIDEO3D_INCLUDE_DIR})

target_link_libraries( guacamole-video3d guacamole ${TurboJPEG_LIBRARIES})

IF (MSVC)
	set_target_properties(guacamole-video3d PROPERTIES COMPILE_FLAGS "-D GUA_VIDEO3D_LIBRARY")
ENDIF (MSVC)

###############################################################################
# install 
###############################################################################

IF (MSVC)
  install (TARGETS guacamole-video3d
    CONFIGURATIONS Release
    RUNTIME DESTINATION bin/Release
    LIBRARY DESTINATION lib/Release
    ARCHIVE DESTINATION lib/Release
  )

  install (TARGETS guacamole-video3d
    CONFIGURATIONS Debug
    RUNTIME DESTINATION bin/Debug
    LIBRARY DESTINATION lib/Debug
    ARCHIVE DESTINATION lib/Debug
  )
ELSEIF (UNIX)
  install (TARGETS guacamole-video3d 
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
