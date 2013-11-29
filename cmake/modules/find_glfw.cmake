##############################################################################
# search paths
##############################################################################

message(STATUS "-- checking for GLFW")

FIND_PATH(GLFW_INCLUDE_DIRS GLFW/glfw3.h DOC "Path to GLFW include directory."
  PATHS
  /opt/glfw3/current/include
  /usr/include/
  /usr/local/include/
  /usr/include/GL
  /usr/local/include/GL
)

FIND_LIBRARY(GLFW_LIBRARY_TEMP DOC "Absolute path to GLFW library."
  NAMES glfw3 GLFW3.lib
  PATHS
  /opt/glfw3/current/lib
  /usr/local/lib
  /usr/lib
)

IF(GLFW_LIBRARY_TEMP AND GLFW_INCLUDE_DIRS)
  MESSAGE(STATUS "--  found matching GLFW version")

  GET_FILENAME_COMPONENT(GLFW_LIBRARIES ${GLFW_LIBRARY_TEMP} NAME)
  SET(GLFW_LIBRARIES ${GLFW_LIBRARIES} CACHE STRING "The name of the GLFW library")

  GET_FILENAME_COMPONENT(GLFW_LIBRARY_DIRS ${GLFW_LIBRARY_TEMP} PATH)
  SET(GLFW_LIBRARY_DIRS ${GLFW_LIBRARY_DIRS} CACHE STRING "Where the GLFW Library can be found")

  SET(GLFW_LIBRARY_TEMP "" CACHE INTERNAL "")
ENDIF(GLFW_LIBRARY_TEMP AND GLFW_INCLUDE_DIRS)
