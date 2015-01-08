# - Try to find the point based renderer (PLOD)
#
#  This module defines the following variables
#
#  PLOD_FOUND - Was PLOD found
#  PLOD_INCLUDE_DIRS - the PLOD include directories
#  PLOD_LIBRARIES - Link to this, by default it includes rendering lib
#
#  This module accepts the following variables
#
#  PLOD_ROOT - Can be set to PLOD install path
#

#=============================================================================
# Copyright 2009 Kitware, Inc.
# Copyright 2009 Philip Lowman <philip@yhbt.com>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

SET(PROTOBUF_INCLUDE_SEARCH_DIRS
    ${PLOD_ROOT}
)

SET(PROTOBUF_LIBRARY_SEARCH_DIRS
    ${PLOD_ROOT}/build/common
    ${PLOD_ROOT}/build_codeblocks_rel/common
    ${PLOD_ROOT}/build_codeblocks/common
)


macro(_PLOD_APPEND_LIBRARIES _list _release)
   set(_debug ${_release}_DEBUG)
   if(${_debug})
      set(${_list} ${${_list}} optimized ${${_release}} debug ${${_debug}})
   else()
      set(${_list} ${${_list}} ${${_release}})
   endif()
endmacro()

# Find includes

find_path(PLOD_COMMON_INCLUDE_DIR 
  NAMES pbr/utils.h
  PATHS
    ${PLOD_ROOT}/common/include
    ${PLOD_ROOT}/common/src
)

find_path(PLOD_RENDERING_INCLUDE_DIR 
  NAMES pbr/ren/ooc_disk_access.h
  PATHS
    ${PLOD_ROOT}/rendering/include
    ${PLOD_ROOT}/rendering/src
)

# Find the libraries

find_library(PLOD_COMMON_LIBRARY
  NAMES pbr_common
  HINTS
    ${PROTOBUF_LIBRARY_SEARCH_DIRS}
)

find_library(PLOD_RENDERING_LIBRARY
  NAMES pbr_rendering
  HINTS
    ${PLOD_ROOT}/build/rendering
    ${PLOD_ROOT}/build_codeblocks_rel/rendering
    ${PLOD_ROOT}/build_codeblocks/rendering
)

SET(PLOD_INCLUDE_DIR "${PLOD_COMMON_INCLUDE_DIR} ${PLOD_RENDERING_INCLUDE_DIR}" CACHE PATH "PLOD include dirs")

# handle the QUIETLY and REQUIRED arguments and set PLOD_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PLOD DEFAULT_MSG
    PLOD_COMMON_LIBRARY PLOD_RENDERING_LIBRARY
    PLOD_COMMON_INCLUDE_DIR PLOD_RENDERING_INCLUDE_DIR)

if(PLOD_FOUND)
   set(PLOD_INCLUDE_DIRS ${PLOD_COMMON_INCLUDE_DIR} ${PLOD_RENDERING_INCLUDE_DIR})
   _PLOD_APPEND_LIBRARIES(PLOD_LIBRARIES PLOD_COMMON_LIBRARY)
   _PLOD_APPEND_LIBRARIES(PLOD_LIBRARIES PLOD_RENDERING_LIBRARY)
endif()

