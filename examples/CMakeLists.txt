# guacamole examples
IF (GUACAMOLE_EXAMPLES)

  # input requires GLFW3
  IF (${GUACAMOLE_GLFW3})
    add_subdirectory(clipping)
    add_subdirectory(input)
    add_subdirectory(multicontext)
    add_subdirectory(nice_and_shiny)
    add_subdirectory(oit_demo)
    add_subdirectory(picking)
    add_subdirectory(projective_texturing)
    add_subdirectory(smooth_lighting)
    add_subdirectory(stress_test)
    add_subdirectory(transparency)
    add_subdirectory(tiled_shading_test)
    add_subdirectory(volumetric_point_and_line_rendering)
  ENDIF (${GUACAMOLE_GLFW3})

  IF (${PLUGIN_guacamole-spoints})
    add_subdirectory(avatars_in_industry)
  ENDIF (${PLUGIN_guacamole-spoints})
  
  # skelanim example requires skelanim-plugin and the fbx sdk
  if(${PLUGIN_guacamole-skelanim} AND ${GUACAMOLE_FBX})
    add_subdirectory(animation)
  endif(${PLUGIN_guacamole-skelanim} AND ${GUACAMOLE_FBX})

  # nurbs example requires GLFW3 and nurbs-plugin
  IF (${PLUGIN_guacamole-nurbs} AND ${GUACAMOLE_GLFW3})
    add_subdirectory(nurbs)
  ENDIF (${PLUGIN_guacamole-nurbs} AND ${GUACAMOLE_GLFW3})

  # oculus example requires oculus-plugin
  IF (${PLUGIN_guacamole-oculus})
    add_subdirectory(oculus)
  ENDIF (${PLUGIN_guacamole-oculus})

  IF(${PLUGIN_guacamole-oculus} AND ${PLUGIN_guacamole-plod})
    add_subdirectory(oculus_pitoti)
  ENDIF (${PLUGIN_guacamole-oculus} AND ${PLUGIN_guacamole-plod})

  # vive example requires vive-plugin
  IF (${PLUGIN_guacamole-vive})
    add_subdirectory(vive)
  ENDIF (${PLUGIN_guacamole-vive})

  # plod example requires plod-plugin
  IF (${PLUGIN_guacamole-plod})
    add_subdirectory(plod_shadows)
    add_subdirectory(plod_simple)
    IF (${PLUGIN_guacamole-gui})
      add_subdirectory(depth_cube_map)
    ENDIF()
  ENDIF (${PLUGIN_guacamole-plod})

  # lod example requires lod-plugin
  IF (${PLUGIN_guacamole-lod})
    add_subdirectory(lod_mesh_textured)
    add_subdirectory(lod_simple)
    IF (GUACAMOLE_ENABLE_VIRTUAL_TEXTURING)
      add_subdirectory(lod_mesh_virtually_textured)
    ENDIF (GUACAMOLE_ENABLE_VIRTUAL_TEXTURING)
  ENDIF (${PLUGIN_guacamole-lod})

  # time-varying volume example requires tv_3-plugin
  IF (${PLUGIN_guacamole-tv_3})
    add_subdirectory(time_varying_volume)
  ENDIF (${PLUGIN_guacamole-tv_3})

  # gui example requires gui-plugin
  IF (${PLUGIN_guacamole-gui})
   add_subdirectory(gui)
    # volume example requires volume-plugin and gui-plugin
   IF (${PLUGIN_guacamole-volume})
     add_subdirectory(volume)
   ENDIF (${PLUGIN_guacamole-volume})
  ENDIF (${PLUGIN_guacamole-gui})

  add_subdirectory(tiledwall)

  # video3d example requires glfw and video3d-plugin
  IF (${GUACAMOLE_GLFW3} AND ${PLUGIN_guacamole-video3d})
    add_subdirectory(video3d)
  ENDIF (${GUACAMOLE_GLFW3} AND ${PLUGIN_guacamole-video3d})

  # spoints example requires glfw and spoints-plugin
  IF (${GUACAMOLE_GLFW3} AND ${PLUGIN_guacamole-spoints})
    add_subdirectory(spoints)
  ENDIF (${GUACAMOLE_GLFW3} AND ${PLUGIN_guacamole-spoints})

  IF (GUACAMOLE_ENABLE_PHYSICS)
    add_subdirectory(physics)
  ENDIF (GUACAMOLE_ENABLE_PHYSICS)

  IF (${PLUGIN_guacamole-nrp} AND ${PLUGIN_guacamole-video3d})
    add_subdirectory(nrp_scene)
  ENDIF (${PLUGIN_guacamole-nrp} AND ${PLUGIN_guacamole-video3d})

  IF (GUACAMOLE_ENABLE_VIRTUAL_TEXTURING)
    add_subdirectory(projective_virtual_texturing)
    add_subdirectory(virtual_texturing)
  ENDIF (GUACAMOLE_ENABLE_VIRTUAL_TEXTURING)


ENDIF (GUACAMOLE_EXAMPLES)
