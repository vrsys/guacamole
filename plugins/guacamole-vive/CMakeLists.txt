# dependencies
include(find_openvr)

# determine source and header files
file(GLOB_RECURSE GUACAMOLE_VIVE_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    src/*.cpp
    include/*.h
    include/*.hpp
)

SET(GUACAMOLE_VIVE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-vive includes.")

LINK_DIRECTORIES(${LIB_PATHS} ${OPENVR_LIBRARY_DIRS})

INCLUDE_DIRECTORIES(
    ${INCLUDE_PATHS}
    ${GUACAMOLE_SOURCE_DIR}
    ${OPENVR_INCLUDE_DIR}
    ${GUACAMOLE_VIVE_INCLUDE_DIR}
)

ADD_LIBRARY(guacamole-vive SHARED ${GUACAMOLE_VIVE_SRC})

ADD_DEPENDENCIES(guacamole-vive guacamole)

IF (MSVC)
    set_target_properties(guacamole-vive PROPERTIES COMPILE_FLAGS "-D GUA_VIVE_LIBRARY")
ENDIF (MSVC)

IF (UNIX)
    LIST(APPEND LIBS udev Xinerama pthread guacamole optimized ${OPENVR_LIBRARY} debug ${OPENVR_LIBRARY_DEBUG})
ELSEIF (MSVC)
    LIST(APPEND LIBS winmm.lib guacamole ${OPENVR_LIBRARY})
ENDIF(UNIX)

find_package (Threads)

if (MSVC)
    target_link_libraries(
        guacamole-vive
        ${LIBS}
        optimized ${FREEIMAGE_LIBRARY}
        debug ${FREEIMAGE_LIBRARY_DEBUG}
        optimized ${FREEIMAGE_PLUS_LIBRARY}
        debug ${FREEIMAGE_PLUS_LIBRARY_DEBUG}
        ${CMAKE_THREAD_LIBS_INIT}
    )
elseif (UNIX)
    target_link_libraries(
        guacamole-vive
        ${LIBS}
        ${CMAKE_THREAD_LIBS_INIT}
    )
endif (MSVC)

###############################################################################
# install
###############################################################################

IF (MSVC)
    install (TARGETS guacamole-vive
        CONFIGURATIONS Release
        RUNTIME DESTINATION bin/Release
        LIBRARY DESTINATION lib/Release
        ARCHIVE DESTINATION lib/Release
    )

    install (TARGETS guacamole-vive
        CONFIGURATIONS Debug
        RUNTIME DESTINATION bin/Debug
        LIBRARY DESTINATION lib/Debug
        ARCHIVE DESTINATION lib/Debug
    )
ELSEIF (UNIX)
    install (TARGETS guacamole-vive
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
