##############################################################################
# search paths
##############################################################################
SET(PCC_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/libpcc/include
  ${PCC_INCLUDE_DIRS}
  /opt/libpcc/include
)

SET(PCC_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/libpcc/lib
  ${PCC_LIBRARY_DIRS}
  /opt/libpcc/lib
)

##############################################################################
# search
##############################################################################
find_path (PCC_INCLUDE_DIRS
           NAMES PointCloudGridEncoder.hpp
           HINTS
           ${PCC_INCLUDE_SEARCH_DIRS}
           NO_DEFAULT_PATH
           NO_CMAKE_ENVIRONMENT_PATH
           NO_CMAKE_SYSTEM_PATH
           NO_SYSTEM_ENVIRONMENT_PATH
           NO_CMAKE_PATH
           CMAKE_FIND_FRAMEWORK NEVER
           PATHS
)

IF (UNIX)
  find_library (PCC_LIBRARY
                NAMES pcc
                PATHS ${PCC_LIBRARY_SEARCH_DIRS}
                )
  SET(PCC_LIBRARY_DEBUG ${PCC_LIBRARY} CACHE STRING "pcc libraries.")   
ENDIF()
