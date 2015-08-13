# guacamole examples

set(LIB_PATH
  ${SCHISM_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  ${BULLET_LIBRARY_DIRS}
  ${GUACAMOLE_LIBRARY_DIRS}
  ${ASSIMP_LIBRARY_DIRS}
  ${FBX_LIBRARY_DIRS}
  ${JSON_LIBRARY_DIRS}
  )
link_directories(${LIB_PATH})

set(LIBS
  ${SCHISM_LIBRARIES}
  ${Boost_LIBRARIES}
  ${BULLET_LIBRARIES}
  ${GUACAMOLE_LIBRARIES}
  ${FBX_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${JSON_LIBRARIES}
  )
#link_libraries(${LIBS})

set(INCLUDE_PATHS
  ${SCHISM_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
  ${GUACAMOLE_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIR}
  ${FBX_INCLUDE_DIR}
  ${JSON_INCLUDE_DIRS}
  )

IF (GUACAMOLE_EXAMPLES)

  # input requires GLFW3
  IF (${GUACAMOLE_GLFW3})
    add_subdirectory(input)
    add_subdirectory(projective_texturing)
    add_subdirectory(nice_and_shiny)
    add_subdirectory(smooth_lighting)
    add_subdirectory(stress_test)
    add_subdirectory(transparency)
    add_subdirectory(oit_demo)
    add_subdirectory(tiled_shading_test)
    add_subdirectory(clipping)
  ENDIF (${GUACAMOLE_GLFW3})
  add_subdirectory(screenshot)

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
    add_subdirectory(oculusSDK2_Pitoti)
  ENDIF (${PLUGIN_guacamole-oculus} AND ${PLUGIN_guacamole-plod})

  # plod example requires plod-plugin
  IF (${PLUGIN_guacamole-plod})
    add_subdirectory(plod)
  ENDIF (${PLUGIN_guacamole-plod})
  
  # gui example requires gui-plugin
  IF (${PLUGIN_guacamole-gui})
   add_subdirectory(gui)
    # volume example requires volume-plugin and gui-plugin
   IF (${PLUGIN_guacamole-volume})
     add_subdirectory(volume)
   ENDIF (${PLUGIN_guacamole-volume})
  ENDIF (${PLUGIN_guacamole-gui})


  # multicontext example requires plod-plugin
  IF (${PLUGIN_guacamole-plod})
    add_subdirectory(multicontext)
  ENDIF (${PLUGIN_guacamole-plod})

  # video3d example requires glfw and video3d-plugin
  IF (${GUACAMOLE_GLFW3} AND ${PLUGIN_guacamole-video3d})
      add_subdirectory(video3d)
    ENDIF (${GUACAMOLE_GLFW3} AND ${PLUGIN_guacamole-video3d})

ENDIF (GUACAMOLE_EXAMPLES)
