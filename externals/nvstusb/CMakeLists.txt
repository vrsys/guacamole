###############################################################################
# determine source files
###############################################################################
FILE(GLOB         NVSTUSB_SRC           RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.c)

###############################################################################
# determine header files
###############################################################################
FILE(GLOB         NVSTUSB_INC           RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h)

ADD_LIBRARY( nvstusb SHARED
  ${NVSTUSB_INC}
  ${NVSTUSB_SRC}
)

TARGET_LINK_LIBRARIES( nvstusb general ${LIBUSB_LIBRARIES})
SET_TARGET_PROPERTIES( nvstusb PROPERTIES LINKER_LANGUAGE C)


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
  install (TARGETS nvstusb
    RUNTIME DESTINATION lib_external
    LIBRARY DESTINATION lib_external
    ARCHIVE DESTINATION lib_external
  )
ENDIF (MSVC)

# header files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION src_external/ FILES_MATCHING PATTERN "*.h")
