###############################################################################
# determine source files
###############################################################################
FILE(GLOB         LZ4_SRC           RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.c)

###############################################################################
# determine header files
###############################################################################
FILE(GLOB         LZ4_INC           RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h)

ADD_LIBRARY(lz4 STATIC
  ${LZ4_SRC}
  ${LZ4_INC}
)

SET_TARGET_PROPERTIES(lz4 PROPERTIES LINKER_LANGUAGE C)
SET_PROPERTY(TARGET lz4 PROPERTY POSITION_INDEPENDENT_CODE ON)

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
  install (TARGETS lz4
    RUNTIME DESTINATION lib_external
    LIBRARY DESTINATION lib_external
    ARCHIVE DESTINATION lib_external
  )
ENDIF (MSVC)

# header files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION src_external/ FILES_MATCHING PATTERN "*.h")
