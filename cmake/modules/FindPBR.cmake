# - Try to find the point based renderer (PBR)
#
#  This module defines the following variables
#
#  PBR_FOUND - Was PBR found
#  PBR_INCLUDE_DIRS - the PBR include directories
#  PBR_LIBRARIES - Link to this, by default it includes rendering lib
#
#  This module accepts the following variables
#
#  PBR_ROOT - Can be set to PBR install path
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

macro(_PBR_APPEND_LIBRARIES _list _release)
   set(_debug ${_release}_DEBUG)
   if(${_debug})
      set(${_list} ${${_list}} optimized ${${_release}} debug ${${_debug}})
   else()
      set(${_list} ${${_list}} ${${_release}})
   endif()
endmacro()

# Find includes

find_path(PBR_COMMON_INCLUDE_DIR 
  NAMES pbr/utils.h
  HINTS
    ${PBR_ROOT}/common/include
    ${PBR_ROOT}/common/src
)

find_path(PBR_RENDERING_INCLUDE_DIR 
  NAMES pbr/ren/renderer.h
  HINTS
    ${PBR_ROOT}/rendering/include
    ${PBR_ROOT}/rendering/src
)

# Find the libraries

find_library(PBR_COMMON_LIBRARY
  NAMES pbr_common
  HINTS
    ${PBR_ROOT}/build/common
    ${PBR_ROOT}/build_codeblocks_rel/common
    ${PBR_ROOT}/build_codeblocks/common
)

find_library(PBR_RENDERING_LIBRARY
  NAMES pbr_rendering
  HINTS
    ${PBR_ROOT}/build/rendering
    ${PBR_ROOT}/build_codeblocks_rel/rendering
    ${PBR_ROOT}/build_codeblocks/rendering
)

# handle the QUIETLY and REQUIRED arguments and set PBR_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PBR DEFAULT_MSG
    PBR_COMMON_LIBRARY PBR_RENDERING_LIBRARY
    PBR_COMMON_INCLUDE_DIR PBR_RENDERING_INCLUDE_DIR)

if(PBR_FOUND)
   set(PBR_INCLUDE_DIRS ${PBR_COMMON_INCLUDE_DIR} ${PBR_RENDERING_INCLUDE_DIR})
   _PBR_APPEND_LIBRARIES(PBR_LIBRARIES PBR_COMMON_LIBRARY)
   _PBR_APPEND_LIBRARIES(PBR_LIBRARIES PBR_RENDERING_LIBRARY)
endif()

