# guacamole examples

IF (GUACAMOLE_EXAMPLES)
    add_subdirectory(nice_and_shiny)
    add_subdirectory(volume)
    add_subdirectory(gui)
    add_subdirectory(warping)
    add_subdirectory(input)
    add_subdirectory(oit_demo)
    add_subdirectory(lod_simple)
    add_subdirectory(plod_simple)
    add_subdirectory(plod_shadows)
    add_subdirectory(tiled_shading_test)
    add_subdirectory(video3d)
    add_subdirectory(stress_test)
    add_subdirectory(smooth_lighting)
    add_subdirectory(projective_texturing)
    add_subdirectory(animation)
    add_subdirectory(clipping)
    add_subdirectory(multicontext)

  # # input requires GLFW3
  # IF (${GUACAMOLE_GLFW3})
  #   add_subdirectory(input)
  #   add_subdirectory(projective_texturing)
  #   add_subdirectory(nice_and_shiny)
  #   add_subdirectory(smooth_lighting)
  #   add_subdirectory(stress_test)
  #   add_subdirectory(transparency)
  #   add_subdirectory(oit_demo)
  #   add_subdirectory(tiled_shading_test)
  #   add_subdirectory(clipping)
  # ENDIF (${GUACAMOLE_GLFW3})

  # add_subdirectory(screenshot)

  # # skelanim example requires skelanim-plugin and the fbx sdk
  # if(${PLUGIN_guacamole-skelanim} AND ${GUACAMOLE_FBX})
  #   add_subdirectory(animation)
  # endif(${PLUGIN_guacamole-skelanim} AND ${GUACAMOLE_FBX})

  # # nurbs example requires GLFW3 and nurbs-plugin
  # IF (${PLUGIN_guacamole-nurbs} AND ${GUACAMOLE_GLFW3})
  #   # add_subdirectory(nurbs)
  # ENDIF (${PLUGIN_guacamole-nurbs} AND ${GUACAMOLE_GLFW3})

# <<<<<<< HEAD
#   # # oculus example requires oculus-plugin
#   # IF (${PLUGIN_guacamole-oculus})
#   #   add_subdirectory(oculus)
#   # ENDIF (${PLUGIN_guacamole-oculus})

#   # IF(${PLUGIN_guacamole-oculus} AND ${PLUGIN_guacamole-plod})
#   #   add_subdirectory(oculus_pitoti)
#   # ENDIF (${PLUGIN_guacamole-oculus} AND ${PLUGIN_guacamole-plod})

#   # # plod example requires plod-plugin
#   # IF (${PLUGIN_guacamole-plod})
#   #   LIST(APPEND LIBS -lguacamole-plod)
#   # ENDIF (${PLUGIN_guacamole-plod})
  
#   # IF (${PLUGIN_guacamole-plod} AND ${GUACAMOLE_GLFW3})
#   #   add_subdirectory(plod)
#   # ENDIF (${PLUGIN_guacamole-plod} AND ${GUACAMOLE_GLFW3})

#   # IF (${PLUGIN_guacamole-video3d})
#   #   add_subdirectory(video3d)
#   # ENDIF (${PLUGIN_guacamole-video3d})

#   # add_subdirectory(warping)
# =======
#   # plod example requires plod-plugin
#   IF (${PLUGIN_guacamole-plod})
#     add_subdirectory(plod_simple)
#     add_subdirectory(plod_shadows)
#   ENDIF (${PLUGIN_guacamole-plod})

#   # lod example requires lod-plugin
#   IF (${PLUGIN_guacamole-lod})
#     add_subdirectory(lod_simple)
#   ENDIF (${PLUGIN_guacamole-lod})

#   # gui example requires gui-plugin
#   IF (${PLUGIN_guacamole-gui})
#    add_subdirectory(gui)
#     # volume example requires volume-plugin and gui-plugin
#    IF (${PLUGIN_guacamole-volume})
#      add_subdirectory(volume)
#    ENDIF (${PLUGIN_guacamole-volume})
#   ENDIF (${PLUGIN_guacamole-gui})
# >>>>>>> upstream/master

  # # gui example requires gui-plugin
  # IF (${PLUGIN_guacamole-gui})
  #   add_subdirectory(gui)
  #   # volume example requires volume-plugin and gui-plugin
  #   IF (${PLUGIN_guacamole-volume})
  #     add_subdirectory(volume)
  #   ENDIF (${PLUGIN_guacamole-volume})
  # ENDIF (${PLUGIN_guacamole-gui})

  # # multicontext example requires plod-plugin
  # IF (${PLUGIN_guacamole-plod})
  #   add_subdirectory(multicontext)
  # ENDIF (${PLUGIN_guacamole-plod})

  # # video3d example requires glfw and video3d-plugin
  # IF (${GUACAMOLE_GLFW3} AND ${PLUGIN_guacamole-video3d})
  #     add_subdirectory(video3d)
  # ENDIF (${GUACAMOLE_GLFW3} AND ${PLUGIN_guacamole-video3d})

ENDIF (GUACAMOLE_EXAMPLES)
