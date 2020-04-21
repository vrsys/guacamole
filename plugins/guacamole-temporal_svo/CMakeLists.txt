# dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# determine source and header files
file(GLOB_RECURSE GUACAMOLE_TEMPORAL_SVO_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    src/*.cpp
    include/*.h
    include/*.hpp
)

SET(GUACAMOLE_TEMPORAL_SVO_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-temporal_svo includes.")

LINK_DIRECTORIES(${LIB_PATHS})

INCLUDE_DIRECTORIES( ${INCLUDE_PATHS}
                     ${GUACAMOLE_SOURCE_DIR}
                     ${GUACAMOLE_TEMPORAL_SVO_INCLUDE_DIR}
)


ADD_LIBRARY( guacamole-temporal_svo SHARED
    ${GUACAMOLE_TEMPORAL_SVO_SRC}
)

TARGET_LINK_LIBRARIES( guacamole-temporal_svo
                       guacamole
                       general ${GL_LIBRARIES})

ADD_DEPENDENCIES(guacamole-temporal_svo guacamole)

IF (MSVC)
	set_target_properties(guacamole-temporal_svo PROPERTIES COMPILE_FLAGS "-D GUA_TEMPORAL_SVO_LIBRARY")
ENDIF (MSVC)

###############################################################################
# install
###############################################################################

IF (MSVC)
  install (TARGETS guacamole-temporal_svo
    CONFIGURATIONS Release
    RUNTIME DESTINATION bin/Release
    LIBRARY DESTINATION lib/Release
    ARCHIVE DESTINATION lib/Release
  )

  install (TARGETS guacamole-temporal_svo
    CONFIGURATIONS Debug
    RUNTIME DESTINATION bin/Debug
    LIBRARY DESTINATION lib/Debug
    ARCHIVE DESTINATION lib/Debug
  )
ELSEIF (UNIX)
  install (TARGETS guacamole-temporal_svo
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
