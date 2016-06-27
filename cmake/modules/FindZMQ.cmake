##############################################################################
# search paths
##############################################################################
SET(ZMQ_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/zmq/include
  ${ZMQ_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_SEARCH_DIR}
  /opt/zmq/current/include
)

SET(ZMQ_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/zmq/lib
  ${ZMQ_LIBRARY_DIRS}
  ${ZMQ_LIBRARY_SEARCH_DIR}
  /opt/zmq/current/lib
)

##############################################################################
# search
##############################################################################
find_path (ZMQ_INCLUDE_DIRS
           NAMES zmq.h
           HINTS
           ${ZMQ_INCLUDE_SEARCH_DIRS}
           NO_DEFAULT_PATH
           NO_CMAKE_ENVIRONMENT_PATH
           NO_CMAKE_SYSTEM_PATH
           NO_SYSTEM_ENVIRONMENT_PATH
           NO_CMAKE_PATH
           CMAKE_FIND_FRAMEWORK NEVER
           PATHS
)

IF (MSVC)

  FILE(READ ${ZMQ_INCLUDE_DIRS}/zmq.h _ZMQ_HEADER_CONTENTS)
  
  STRING(REGEX REPLACE ".*#define ZMQ_VERSION_MAJOR ([0-9]+).*" "\\1"      _ZMQ_MAJOR_VERSION   "${_ZMQ_HEADER_CONTENTS}")
  STRING(REGEX REPLACE ".*#define ZMQ_VERSION_MINOR ([0-9]+).*" "\\1"      _ZMQ_MINOR_VERSION   "${_ZMQ_HEADER_CONTENTS}")
  STRING(REGEX REPLACE ".*#define ZMQ_VERSION_PATCH ([0-9]+).*" "\\1"      _ZMQ_PATCH_VERSION   "${_ZMQ_HEADER_CONTENTS}")

  set(ZMQ_LIBRARY_NAME "libzmq-v${COMPILER_SUFFIX_VERSION}-mt-${_ZMQ_MAJOR_VERSION}_${_ZMQ_MINOR_VERSION}_${_ZMQ_PATCH_VERSION}.lib")
  set(ZMQ_LIBRARY_NAME_DEBUG "libzmq-v${COMPILER_SUFFIX_VERSION}-mt-gd-${_ZMQ_MAJOR_VERSION}_${_ZMQ_MINOR_VERSION}_${_ZMQ_PATCH_VERSION}.lib")

  find_library (ZMQ_LIBRARY
                  NAMES ${ZMQ_LIBRARY_NAME} libzmq
                  PATH_SUFFIXES release
                  PATHS ${ZMQ_LIBRARY_SEARCH_DIRS}
                  )
  find_library (ZMQ_LIBRARY_DEBUG
                  NAMES ${ZMQ_LIBRARY_NAME_DEBUG} libzmq
                  PATH_SUFFIXES debug
                  PATHS ${ZMQ_LIBRARY_SEARCH_DIRS}
                  )

ELSEIF (UNIX)
  find_library (ZMQ_LIBRARY
                NAMES zmq
                PATHS ${ZMQ_LIBRARY_SEARCH_DIRS}
                )
  SET(ZMQ_LIBRARY_DEBUG ${ZMQ_LIBRARY} CACHE STRING "zmq libraries.")   
ENDIF()