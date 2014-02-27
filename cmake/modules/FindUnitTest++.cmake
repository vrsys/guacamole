# - Try to find UnitTest++
# Once done this will define
#  UNITTEST++_FOUND - System has UnitTest++
#  UNITTEST++_INCLUDE_DIRS - The UnitTest++ include directories
#  UNITTEST++_LIBRARIES - The libraries needed to use UnitTest++
find_path(UNITTEST++_INCLUDE_DIR UnitTest++.h
            HINTS ${CMAKE_SOURCE_DIR}/externals/UnitTest++/include
            PATHS /usr/include/unittest++
            PATH_SUFFIXES UnitTest++ unittest++
            )

find_library(UNITTEST++_LIBRARY
            NAMES UnitTest++ UnitTest++.vsnet2005 UnitTest++.vsnet2008
            HINTS ${CMAKE_SOURCE_DIR}/externals/UnitTest++/lib
            PATHS /usr/lib
            )

#find_library(UNITTEST++_LIBRARY_DEBUG
#           NAMES UnitTest++D UnitTest++.vsnet2005 UnitTest++.vsnet2008
#           HINTS ${CMAKE_SOURCE_DIR}/externals/UnitTest++/lib
#           PATHS /usr/lib
#           )

set(UNITTEST++_LIBRARIES ${UNITTEST++_LIBRARY} )
set(UNITTEST++_INCLUDE_DIRS ${UNITTEST++_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(UnitTest++  DEFAULT_MSG
                                  UNITTEST++_LIBRARY UNITTEST++_INCLUDE_DIR)

mark_as_advanced(UNITTEST++_INCLUDE_DIR UNITTEST++_LIBRARY )
