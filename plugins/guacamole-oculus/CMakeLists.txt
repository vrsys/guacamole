# dependencies
include(find_ovr)

# determine source and header files
file(GLOB_RECURSE GUACAMOLE_OCULUS_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    src/*.cpp
    include/*.h
    include/*.hpp
)

SET(GUACAMOLE_OCULUS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-oculus includes.")

LINK_DIRECTORIES(${LIB_PATHS} ${OVR_LIBRARY_DIRS})

add_library( guacamole-oculus SHARED
    ${GUACAMOLE_OCULUS_SRC}
)

target_include_directories(guacamole-oculus PUBLIC
		     ${OVR_INCLUDE_DIR}
		     ${GUACAMOLE_OCULUS_INCLUDE_DIR}
)

IF (MSVC)
  set_target_properties(guacamole-oculus PROPERTIES COMPILE_FLAGS "-D GUA_OCULUS_LIBRARY")
ENDIF (MSVC)

IF (UNIX)
  LIST(APPEND LIBS udev Xinerama pthread guacamole optimized ${OVR_LIBRARY} debug ${OVR_LIBRARY_DEBUG})
ELSEIF (MSVC)
  LIST(APPEND LIBS winmm.lib guacamole ${OVR_LIBRARY})
ENDIF(UNIX)

find_package (Threads)

if (MSVC)
  target_link_libraries( guacamole-oculus
                         ${LIBS}
                         optimized ${FREEIMAGE_LIBRARY}
                         debug ${FREEIMAGE_LIBRARY_DEBUG}
                         optimized ${FREEIMAGE_PLUS_LIBRARY}
                         debug ${FREEIMAGE_PLUS_LIBRARY_DEBUG}
                         ${CMAKE_THREAD_LIBS_INIT} )
elseif (UNIX)
  target_link_libraries( guacamole-oculus
                         ${LIBS}
                         ${CMAKE_THREAD_LIBS_INIT} )
endif (MSVC)

###############################################################################
# install
###############################################################################

IF (MSVC)
  install (TARGETS guacamole-oculus
    CONFIGURATIONS Release
    RUNTIME DESTINATION bin/Release
    LIBRARY DESTINATION lib/Release
    ARCHIVE DESTINATION lib/Release
  )

  install (TARGETS guacamole-oculus
    CONFIGURATIONS Debug
    RUNTIME DESTINATION bin/Debug
    LIBRARY DESTINATION lib/Debug
    ARCHIVE DESTINATION lib/Debug
  )
ELSEIF (UNIX)
  install (TARGETS guacamole-oculus
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
