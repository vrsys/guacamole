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

INCLUDE_DIRECTORIES( ${INCLUDE_PATHS}
                     ${GUACAMOLE_SOURCE_DIR}
                     ${OVR_INCLUDE_DIRS}
                     ${GUACAMOLE_OCULUS_INCLUDE_DIR}
)

ADD_LIBRARY( guacamole-oculus SHARED
    ${GUACAMOLE_OCULUS_SRC}
)

ADD_DEPENDENCIES(guacamole-oculus guacamole)

IF (MSVC)
	set_target_properties(guacamole-oculus PROPERTIES COMPILE_FLAGS "-D GUA_OCULUS_LIBRARY")
ENDIF (MSVC)

IF (UNIX)
    LIST(APPEND LIBS udev Xinerama guacamole ${OVR_LIBRARIES})
ELSEIF (MSVC)
    LIST(APPEND LIBS winmm.lib guacamole ${OVR_LIBRARIES})
ENDIF(UNIX)

find_package (Threads)

target_link_libraries( guacamole-oculus debug ${LIBS} ${CMAKE_THREAD_LIBS_INIT} optimized ${LIBS} ${CMAKE_THREAD_LIBS_INIT})

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
