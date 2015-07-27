# - Try to find the FBX SDK
# Once done this will define
#  FBX_FOUND - System has the FBX SDK
#  FBX_INCLUDE_DIRS - The the FBX SDK include directories
#  FBX_LIBRARIES - The libraries needed to use the FBX SDK
find_path(FBX_INCLUDE_DIR fbxsdk.h
            HINTS 
             ${CMAKE_SOURCE_DIR}/externals/fbx20151/include
			 ${CMAKE_SOURCE_DIR}/externals/inc/fbx
            PATHS
             /usr/include/fbx20151
             ${GUACAMOLE_EXT_DIR}/fbx20151/include
             /opt/project_animation/fbx20151/include
             /opt/fbx20151/include
            PATH_SUFFIXES FBX fbx fbxdsk FBXSDK
            )

IF (MSVC)
	SET(FBX_LIBRARY_NAME libfbxsdk-md.lib)			
ELSEIF (UNIX)
	SET(FBX_LIBRARY_NAME libfbxsdk.a )			
ENDIF(MSVC)
			
find_library(FBX_LIBRARY
            NAMES ${FBX_LIBRARY_NAME}
            HINTS 
             ${CMAKE_SOURCE_DIR}/externals/fbx20151/lib/gcc4/x64/release
			 ${CMAKE_SOURCE_DIR}/externals/lib/release
            PATHS
             /usr/lib
             /opt/project_animation/fbx20151/lib/gcc4/x64/release
             /opt/fbx20151/lib/gcc4/x64/release
            )

find_library(FBX_LIBRARY_DEBUG
            NAMES ${FBX_LIBRARY_NAME}
            HINTS 
             ${CMAKE_SOURCE_DIR}/externals/fbx20151/lib/gcc4/x64/debug
			 ${CMAKE_SOURCE_DIR}/externals/lib/debug
            PATHS 
             /usr/lib
             /opt/project_animation/fbx20151/lib/gcc4/x64/debug
             /opt/fbx20151/lib/gcc4/x64/debug
            )

# set variables if found
if (FBX_INCLUDE_DIR AND FBX_LIBRARY_RELEASE)
  set(FBX_FOUND TRUE)
endif()
if (FBX_LIBRARY_RELEASE)
  set (FBX_LIBRARY ${FBX_LIBRARY_RELEASE})
endif()
# if (FBX_LIBRARY_DEBUG AND FBX_LIBRARY_RELEASE)
#   set (FBX_LIBRARY debug ${FBX_LIBRARY_DEBUG} optimized ${FBX_LIBRARY_RELEASE} )
# endif()

# output if search was successfull
if (FBX_FOUND)
  MESSAGE(STATUS "-- Found FBX SDK ${FBX_LIBRARIES}")
endif()
if (!FBX_FOUND)
  MESSAGE(STATUS "-- Did not find FBX SDK")
endif()

# set variables that are used on cmakelist calling this script
set(FBX_LIBRARIES ${FBX_LIBRARY} )
set(FBX_LIBRARIES_DEBUG ${FBX_LIBRARY_DEBUG} )
set(FBX_INCLUDE_DIRS ${FBX_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FBX DEFAULT_MSG
                                  FBX_LIBRARY FBX_INCLUDE_DIR)


# Mark the named cached variables as advanced. An advanced variable will not be
# displayed in any of the cmake GUIs unless the show advanced option is on.
mark_as_advanced(FBX_INCLUDE_DIR FBX_LIBRARY )
