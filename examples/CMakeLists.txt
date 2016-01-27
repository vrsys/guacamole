# guacamole examples

IF (GUACAMOLE_EXAMPLES)

  add_subdirectory(stress_test)
  add_subdirectory(multicontext)

  # requires GLFW3
  IF (${GUACAMOLE_GLFW3})
    add_subdirectory(nice_and_shiny)
    add_subdirectory(input)
    add_subdirectory(oit_demo)
    add_subdirectory(tiled_shading_test)
    add_subdirectory(smooth_lighting)
    add_subdirectory(projective_texturing)
    add_subdirectory(clipping)
    add_subdirectory(warping_simple)

    # skelanim example requires skelanim-plugin and the fbx sdk
    if(${PLUGIN_guacamole-skelanim} AND ${GUACAMOLE_FBX})
      add_subdirectory(animation)
    endif(${PLUGIN_guacamole-skelanim} AND ${GUACAMOLE_FBX})

    # nurbs example requires GLFW3 and nurbs-plugin
    IF (${PLUGIN_guacamole-nurbs})
      add_subdirectory(nurbs)
    ENDIF (${PLUGIN_guacamole-nurbs})

    # plod example requires plod-plugin
    IF (${PLUGIN_guacamole-plod})
      add_subdirectory(plod_simple)
      add_subdirectory(plod_shadows)
    ENDIF (${PLUGIN_guacamole-plod})

     # lod example requires lod-plugin
    IF (${PLUGIN_guacamole-lod})
      add_subdirectory(lod_simple)
    ENDIF (${PLUGIN_guacamole-lod})

    IF (${PLUGIN_guacamole-video3d})
      add_subdirectory(video3d)
    ENDIF (${PLUGIN_guacamole-video3d})

    IF(${PLUGIN_guacamole-oculus} AND ${PLUGIN_guacamole-plod})
      add_subdirectory(oculus_pitoti)
    ENDIF (${PLUGIN_guacamole-oculus} AND ${PLUGIN_guacamole-plod})

    IF(${PLUGIN_guacamole-oculus})
      add_subdirectory(oculus)
    ENDIF (${PLUGIN_guacamole-oculus})

    # gui example requires gui-plugin
    IF (${PLUGIN_guacamole-gui})
      add_subdirectory(warping)
      add_subdirectory(gui)
      # volume example requires volume-plugin and gui-plugin
      IF (${PLUGIN_guacamole-volume})
        add_subdirectory(volume)
      ENDIF (${PLUGIN_guacamole-volume})
    ENDIF (${PLUGIN_guacamole-gui})


  ENDIF (${GUACAMOLE_GLFW3})

ENDIF (GUACAMOLE_EXAMPLES)
