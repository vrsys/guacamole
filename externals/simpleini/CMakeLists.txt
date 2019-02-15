###############################################################################
# determine source files
###############################################################################
FILE(GLOB         SIMPLEINI_SRC           RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.c)

###############################################################################
# determine header files
###############################################################################
FILE(GLOB         SIMPLEINI_INC           RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h)

ADD_LIBRARY( simpleini SHARED
        ${SIMPLEINI_INC}
        ${SIMPLEINI_SRC}
        )


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
    install (TARGETS simpleini
            RUNTIME DESTINATION lib_external
            LIBRARY DESTINATION lib_external
            ARCHIVE DESTINATION lib_external
            )
ENDIF (MSVC)

# header files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION src_external/ FILES_MATCHING PATTERN "*.h")
