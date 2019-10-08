# dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# determine source and header files
file(GLOB_RECURSE GUA_SKELANIM_SRC
    src/*.cpp
)

set(GUA_SKELANIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-skelanim includes.")
file(GLOB_RECURSE GUA_SKELANIM_INC
    ${GUA_SKELANIM_INCLUDE_DIR}/*.h
    ${GUA_SKELANIM_INCLUDE_DIR}/*.hpp
)

add_library(guacamole-skelanim SHARED
             ${GUA_SKELANIM_SRC}
             ${GUA_SKELANIM_INC}
        ../guacamole-nrp/include/gua/nrp/platform.hpp)

target_include_directories(guacamole-skelanim PUBLIC ${GUA_SKELANIM_INCLUDE_DIR})

target_link_libraries(guacamole-skelanim guacamole)

IF (MSVC)
  set_target_properties(guacamole-skelanim PROPERTIES COMPILE_FLAGS "-D GUA_SKELANIM_LIBRARY")
ENDIF (MSVC)

SOURCE_GROUP("skelanim" FILES ${GUA_SKELANIM_SRC})
SOURCE_GROUP("skelanim" FILES ${GUA_SKELANIM_INC})
###############################################################################
# install
###############################################################################

IF (MSVC)
  install (TARGETS guacamole-skelanim
    CONFIGURATIONS Release
    RUNTIME DESTINATION bin/Release
    LIBRARY DESTINATION lib/Release
    ARCHIVE DESTINATION lib/Release
  )

  install (TARGETS guacamole-skelanim
    CONFIGURATIONS Debug
    RUNTIME DESTINATION bin/Debug
    LIBRARY DESTINATION lib/Debug
    ARCHIVE DESTINATION lib/Debug
  )
ELSEIF (UNIX)
  install (TARGETS guacamole-skelanim
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
