###############################################################################
# determine header files
###############################################################################
FILE(GLOB TINY_JPEG_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h)

ADD_LIBRARY(tiny_jpeg STATIC
        ${TINY_JPEG_INC}
        ${TINY_JPEG_SRC}
        )

set_target_properties(tiny_jpeg PROPERTIES LINKER_LANGUAGE CXX)

###############################################################################
# install
###############################################################################

IF (MSVC)
    # install (TARGETS guacamole
    #   CONFIGURATIONS Release
    #   RUNTIME DESTINATION bin/Release
    #   LIBRARY DESTINATION lib/Release
    #   ARCHIVE DESTINATION lib/Release
    # )

    # install (TARGETS guacamole
    #   CONFIGURATIONS Debug
    #   RUNTIME DESTINATION bin/Debug
    #   LIBRARY DESTINATION lib/Debug
    #   ARCHIVE DESTINATION lib/Debug
    # )
ELSEIF (UNIX)
    install(TARGETS tiny_jpeg
            RUNTIME DESTINATION lib_external
            LIBRARY DESTINATION lib_external
            ARCHIVE DESTINATION lib_external
            )
ENDIF (MSVC)

# header files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION src_external/ FILES_MATCHING PATTERN "*.h")
