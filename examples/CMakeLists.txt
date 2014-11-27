# guacamole examples

set(LIB_PATH
  ${SCHISM_LIBRARY_DIRS}
  ${BOOST_LIBRARY_DIRS}
  ${BULLET_LIBRARY_DIRS}
  ${GUACAMOLE_LIBRARY_DIRS}
  ${ASSIMP_LIBRARY_DIRS}
  ${JSON_LIBRARY_DIRS}
  )
link_directories(${LIB_PATH})

set(LIBS
  ${SCHISM_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${BULLET_LIBRARIES}
  ${GUACAMOLE_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${JSON_LIBRARIES}
  )
#link_libraries(${LIBS})

set(INCLUDE_PATHS
  ${SCHISM_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
  ${GUACAMOLE_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIR}
  ${JSON_INCLUDE_DIRS}
  )

IF (GUACAMOLE_EXAMPLES)

  # input requires GLFW3
  IF (${GUACAMOLE_GLFW3})
    add_subdirectory(input)
    add_subdirectory(nice_and_shiny)
    add_subdirectory(stress_test)
  ENDIF (${GUACAMOLE_GLFW3})

  # nurbs example requires GLFW3 and nurbs-plugin
  IF (${PLUGIN_guacamole-nurbs} AND ${GUACAMOLE_GLFW3})
    add_subdirectory(nurbs)
  ENDIF (${PLUGIN_guacamole-nurbs} AND ${GUACAMOLE_GLFW3})

  # oculus example requires oculus-plugin
  IF (${PLUGIN_guacamole-oculus})
    add_subdirectory(oculus)
  ENDIF (${PLUGIN_guacamole-oculus})

  # gui example requires gui-plugin
  IF (${PLUGIN_guacamole-gui})
    add_subdirectory(gui)
    # volume example requires volume-plugin and gui-plugin
    IF (${PLUGIN_guacamole-volume})
      add_subdirectory(volume)
    ENDIF (${PLUGIN_guacamole-volume})
  ENDIF (${PLUGIN_guacamole-gui})


  # multicontext example requires nurbs-plugin
  # IF (${PLUGIN_guacamole-nurbs})
    add_subdirectory(multicontext)
  # ENDIF (${PLUGIN_guacamole-nurbs})

ENDIF (GUACAMOLE_EXAMPLES)
