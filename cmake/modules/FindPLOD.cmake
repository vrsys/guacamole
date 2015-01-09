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

SET(PLOD_INCLUDE_SEARCH_DIRS
    ${PLOD_ROOT}
)

SET(PLOD_LIBRARY_SEARCH_DIRS
    ${PLOD_ROOT}/include
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

find_path(PLOD_INCLUDE_DIR 
  NAMES pbr/utils.h
  PATHS
    ${PLOD_ROOT}/include
)

# Find the libraries

find_library(PLOD_COMMON_LIBRARY
  NAMES pbr_common
  HINTS
    ${PLOD_ROOT}/lib
)

find_library(PLOD_RENDERING_LIBRARY
  NAMES pbr_rendering
  HINTS
    ${PLOD_ROOT}/lib
)

SET(PLOD_INCLUDE_DIR "${PLOD_COMMON_INCLUDE_DIR} ${PLOD_RENDERING_INCLUDE_DIR}" CACHE PATH "PLOD include dirs")

# handle the QUIETLY and REQUIRED arguments and set PLOD_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PLOD DEFAULT_MSG
    PLOD_COMMON_LIBRARY PLOD_RENDERING_LIBRARY
    PLOD_INCLUDE_DIR)

if(PLOD_FOUND)
   _PLOD_APPEND_LIBRARIES(PLOD_LIBRARIES PLOD_COMMON_LIBRARY)
   _PLOD_APPEND_LIBRARIES(PLOD_LIBRARIES PLOD_RENDERING_LIBRARY)
endif()

