# dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(find_awesomium)

# determine source and header files
file(GLOB_RECURSE GUACAMOLE_GUI_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    src/*.cpp
    include/*.h
    include/*.hpp
)

SET(GUACAMOLE_GUI_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-gui includes.")

add_library( guacamole-gui SHARED
    ${GUACAMOLE_GUI_SRC}
)

target_include_directories(guacamole-gui PUBLIC
                     ${AWESOMIUM_INCLUDE_DIR}
                     ${GUACAMOLE_GUI_INCLUDE_DIR}
)

IF (MSVC)
	set_target_properties(guacamole-gui PROPERTIES COMPILE_FLAGS "-D GUA_GUI_LIBRARY")
ENDIF (MSVC)

LIST(APPEND LIBS guacamole ${AWESOMIUM_LIBRARY})

target_link_libraries( guacamole-gui debug ${LIBS} optimized ${LIBS})

###############################################################################
# install
###############################################################################

IF (MSVC)
  install (TARGETS guacamole-gui
    CONFIGURATIONS Release
    RUNTIME DESTINATION bin/Release
    LIBRARY DESTINATION lib/Release
    ARCHIVE DESTINATION lib/Release
  )

  install (TARGETS guacamole-gui
    CONFIGURATIONS Debug
    RUNTIME DESTINATION bin/Debug
    LIBRARY DESTINATION lib/Debug
    ARCHIVE DESTINATION lib/Debug
  )
ELSEIF (UNIX)
  install (TARGETS guacamole-gui
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
