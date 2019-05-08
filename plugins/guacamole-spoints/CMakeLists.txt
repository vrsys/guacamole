# dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# determine source and header files
file(GLOB GUA_SPOINTS_INC RELATIVE                ${CMAKE_CURRENT_SOURCE_DIR} include/gua/spoints/*.hpp)
file(GLOB GUA_SPOINTS_SRC RELATIVE                ${CMAKE_CURRENT_SOURCE_DIR} src/gua/spoints/*.cpp)
file(GLOB_RECURSE GUA_SPOINTS_DETAIL_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/spoints/spoints_geometry/*.hpp include/gua/spoints/spoints_geometry/*.h)
file(GLOB_RECURSE GUA_SPOINTS_DETAIL_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/spoints/spoints_geometry/*.cpp)
FILE(GLOB_RECURSE GUA_SPOINTS_SHADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*)

SOURCE_GROUP("renderer" FILES ${GUA_SPOINTS_INC})
SOURCE_GROUP("renderer" FILES ${GUA_SPOINTS_SRC})
SOURCE_GROUP("detail"   FILES ${GUA_SPOINTS_DETAIL_INC})
SOURCE_GROUP("detail"   FILES ${GUA_SPOINTS_DETAIL_SRC})
SOURCE_GROUP("resources" FILES ${GUA_SPOINTS_SHADERS})

SET(GUACAMOLE_SPOINTS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-spoints includes.")


option (GUACAMOLE_SPOINTS_ENABLE_TURBOJPEG "Set to enable usage of TurboJPEG for decompressing images." OFF)


IF (GUACAMOLE_SPOINTS_ENABLE_TURBOJPEG)
find_package(TurboJPEG REQUIRED)
ENDIF (GUACAMOLE_SPOINTS_ENABLE_TURBOJPEG)

add_library( guacamole-spoints SHARED
    ${GUA_SPOINTS_INC}
    ${GUA_SPOINTS_SRC}
    ${GUA_SPOINTS_DETAIL_INC}
    ${GUA_SPOINTS_DETAIL_SRC}
    ${GUA_SPOINTS_SHADERS}
)

target_include_directories(guacamole-spoints PUBLIC ${GUACAMOLE_SPOINTS_INCLUDE_DIR} ${TurboJPEG_INCLUDE_DIRS} ${FASTLZ_INC})

target_link_libraries(guacamole-spoints guacamole ${TurboJPEG_LIBRARIES} lz4)

IF (MSVC)
	set_target_properties(guacamole-spoints PROPERTIES COMPILE_FLAGS "-D GUA_SPOINTS_LIBRARY")
ENDIF (MSVC)

###############################################################################
# install 
###############################################################################

IF (MSVC)
  install (TARGETS guacamole-spoints
    CONFIGURATIONS Release
    RUNTIME DESTINATION bin/Release
    LIBRARY DESTINATION lib/Release
    ARCHIVE DESTINATION lib/Release
  )

  install (TARGETS guacamole-spoints
    CONFIGURATIONS Debug
    RUNTIME DESTINATION bin/Debug
    LIBRARY DESTINATION lib/Debug
    ARCHIVE DESTINATION lib/Debug
  )
ELSEIF (UNIX)
  install (TARGETS guacamole-spoints 
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
