# - Try to find UnitTest++
# Once done this will define
#  JSON_FOUND - System has json
#  JSON_INCLUDE_DIRS - The json include directories
#  JSON_LIBRARIES - The libraries needed to use json
find_path(JSON_INCLUDE_DIR
            NAMES jsoncpp/json/features.h
            PATHS /usr/include/jsoncpp/json
                  ${GLOBAL_EXT_DIR}/inc/json
                  ${GUACAMOLE_EXT_DIR}/inc/json
            PATH_SUFFIXES jsoncpp json
            )

find_library(JSON_LIBRARY
            NAMES jsoncpp json
            PATHS /usr/lib
                  ${GLOBAL_EXT_DIR}/lib
                  ${GUACAMOLE_EXT_DIR}/lib
            PATH_SUFFIXES release
            )

find_library(JSON_LIBRARY_DEBUG
            NAMES jsoncpp jsond
            PATHS /usr/lib
                  ${GLOBAL_EXT_DIR}/lib
                  ${GUACAMOLE_EXT_DIR}/lib
            PATH_SUFFIXES debug
            )

set(JSON_LIBRARIES ${JSON_LIBRARY} )
set(JSON_LIBRARIES_DEBUG ${JSON_LIBRARY_DEBUG} )
set(JSON_INCLUDE_DIRS ${JSON_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Json DEFAULT_MSG
                                  JSON_LIBRARY JSON_INCLUDE_DIR)

# Mark the named cached variables as advanced. An advanced variable will not be
# displayed in any of the cmake GUIs unless the show advanced option is on.
mark_as_advanced(JSON_INCLUDE_DIR JSON_LIBRARY )
