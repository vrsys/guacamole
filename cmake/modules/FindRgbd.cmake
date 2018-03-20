##############################################################################
# search paths
##############################################################################
SET(RGBD_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/librgbd/include
  ${RGBD_INCLUDE_DIRS}
  /opt/librgbd/current/include
  /mnt/telepresence/rgbd-compression/include
)

SET(RGBDDECOMP_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/librgbd/include
  ${RGBDDECOMP_INCLUDE_DIRS}
  /opt/librgbd/current/include
  /mnt/telepresence/rgbd-compression/src10/decoder/include/
)


SET(RGBD_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/librgbd/lib
  ${RGBD_LIBRARY_DIRS}
  /opt/librgbd/current/lib
  /mnt/telepresence/rgbd-compression/lib
)

SET(RGBDDECOMP_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/librgbd/lib
  ${RGBDDECOMP_LIBRARY_DIRS}
  /opt/librgbd/current/lib
  /mnt/telepresence/rgbd-compression/src10/decoder/lib
)

##############################################################################
# search
##############################################################################
find_path (RGBD_INCLUDE_DIRS
           NAMES RGBDCompressor.h
           HINTS
           ${RGBD_INCLUDE_SEARCH_DIRS}
           NO_DEFAULT_PATH
           NO_CMAKE_ENVIRONMENT_PATH
           NO_CMAKE_SYSTEM_PATH
           NO_SYSTEM_ENVIRONMENT_PATH
           NO_CMAKE_PATH
           CMAKE_FIND_FRAMEWORK NEVER
           PATHS
)

find_path (RGBDDECOMP_INCLUDE_DIRS
           NAMES libswscale/swscale.h
           HINTS
           ${RGBDDECOMP_INCLUDE_SEARCH_DIRS}
           NO_DEFAULT_PATH
           NO_CMAKE_ENVIRONMENT_PATH
           NO_CMAKE_SYSTEM_PATH
           NO_SYSTEM_ENVIRONMENT_PATH
           NO_CMAKE_PATH
           CMAKE_FIND_FRAMEWORK NEVER
           PATHS
)

IF (UNIX)
  find_library (RGBD_LIBRARY
                NAMES RGBDCompression
                PATHS ${RGBD_LIBRARY_SEARCH_DIRS}
                )
                
  # here actually avacode lib
  find_library (RGBDDECOMP_LIBRARY
                NAMES avcodec
                PATHS ${RGBDDECOMP_LIBRARY_SEARCH_DIRS}
                )
  SET(RGBD_LIBRARY_DEBUG ${RGBD_LIBRARY} CACHE STRING "rgbdCompression libraries.")
  SET(RGBDDECOMP_LIBRARY_DEBUG ${RGBDDECOMP_LIBRARY} CACHE STRING "rgbdDecompression libraries.")   
ENDIF()