# - Try to find the FBX SDK
# Once done this will define
#  FBX_FOUND - System has the FBX SDK
#  FBX_INCLUDE_DIRS - The the FBX SDK include directories
#  FBX_LIBRARIES - The libraries needed to use the FBX SDK
find_path(FBX_INCLUDE_DIR fbxsdk.h
            HINTS ${CMAKE_SOURCE_DIR}/externals/fbx/include
            PATHS /usr/include/fbxsdk
            PATH_SUFFIXES FBX fbx fbxdsk FBXSDK
            )

find_library(FBX_LIBRARY
            NAMES libfbxsdk.a 
            HINTS ${CMAKE_SOURCE_DIR}/externals/fbx/lib/gcc4/x64/release
            PATHS /usr/lib
            )
# find_library(FBX_LIBRARY_DEBUG
#             NAMES libfbxsdk.a 
#             HINTS ${CMAKE_SOURCE_DIR}/externals/fbxsdk/lib/gcc4/x64/debug
#             PATHS /usr/lib
#             )

set(FBX_LIBRARIES ${FBX_LIBRARY} )
set(FBX_INCLUDE_DIRS ${FBX_INCLUDE_DIR} )

if (FBX_INCLUDE_DIR AND FBX_LIBRARY_RELEASE)
  set(FBX_FOUND TRUE)
endif()
if (FBX_LIBRARY_RELEASE)
  set (FBX_LIBRARY ${FBX_LIBRARY_RELEASE})
endif()
# if (FBX_LIBRARY_DEBUG AND FBX_LIBRARY_RELEASE)
#   set (FBX_LIBRARY debug ${FBX_LIBRARY_DEBUG} optimized ${FBX_LIBRARY_RELEASE} )
# endif()
if (FBX_FOUND)
  MESSAGE(STATUS "-- Found FBX SDK ${FBX_LIBRARIES}")
endif()
if (!FBX_FOUND)
  MESSAGE(STATUS "-- Did not find FBX SDK")
endif()

mark_as_advanced(FBX_INCLUDE_DIR FBX_LIBRARY )
