################################################################
# Project GUACAMOLE
################################################################

PROJECT(GUACAMOLE CXX)

# version number
set(GUACAMOLE_MAJOR 0)
set(GUACAMOLE_MINOR 1)
set(GUACAMOLE_PATCH 0)
set(GUACAMOLE_VERSION ${GUACAMOLE_MAJOR}.${GUACAMOLE_MINOR}.${GUACAMOLE_PATCH})
set(GUACAMOLE_DESCRIPTION "GUACAMOLE - an astonishing virtual reality engine")
set(GUACAMOLE_HOMEPAGE "http://www.GUACAMOLE.org")
set(GUACAMOLE_EXENAME "GUACAMOLE")
set(GUACAMOLE_PACKAGENAME "GUACAMOLE")

# We require at least version 2.8.0
cmake_minimum_required(VERSION 2.8.0)

find_package(PkgConfig)

# Location where cmake first looks for modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

################################################################
# Configure and find libraries
################################################################

pkg_check_modules(ASSIMP REQUIRED assimp)
pkg_check_modules(JSON REQUIRED jsoncpp)
pkg_check_modules(GL REQUIRED gl)
pkg_check_modules(GLEW REQUIRED glew)

include(find_schism)
include(find_boost)
include(find_ev)
include(find_bullet)

set(LIBS
    ${SCHISM_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${JSON_LIBRARIES}
    ${GL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${EV_LIBRARIES}
    ${BULLET_LIBRARIES}
    #profiler
    #tcmalloc
)
link_libraries(${LIBS})

set(LIB_PATHS
    ${ASSIMP_LIBRARY_DIRS}
    ${JSON_LIBRARY_DIRS}
    ${GL_LIBRARY_DIRS}
    ${GLEW_LIBRARY_DIRS}
)
link_directories(${LIB_PATHS})

set(INCLUDE_PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/build
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SCHISM_INCLUDE_DIRS}
    ${BOOST_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${JSON_INCLUDE_DIRS}
    ${GL_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${EV_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
)
include_directories(${INCLUDE_PATHS})

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-s -O4 --std=c++0x -fpermissive")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall --std=c++0x -fpermissive")

################################################################
# Create GuaRC
################################################################

add_subdirectory(guarc)

################################################################
# Generate RC data
################################################################

# list resource files
file(GLOB_RECURSE RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    resources/*
)

if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/generated/)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/generated)
endif()

if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated)
endif()

ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/R.inl
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/guarc/guarc
    ${CMAKE_CURRENT_SOURCE_DIR}/resources
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/generated/R.inl
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/R.inl
  DEPENDS ${RESOURCES} guarc
)

ADD_CUSTOM_TARGET(CompileResources ALL
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/R.inl
)

################################################################
# Create libraries
################################################################

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
add_subdirectory(src)

################################################################
# Summary
################################################################

message( "" )
message( "Summary:" )
message( " build type: ${CMAKE_BUILD_TYPE}" )
message( "" )
message( " schism:" )
message( "   library: ${SCHISM_LIBRARIES}" )
message( "   include: ${SCHISM_INCLUDE_DIRS}" )
message( "" )
message( " boost:" )
message( "   library: ${BOOST_LIBRARIES}" )
message( "   include: ${BOOST_INCLUDE_DIRS}" )
message( "" )
message( " assimp:" )
message( "   library: ${ASSIMP_LIBRARIES}" )
message( "   include: ${ASSIMP_INCLUDE_DIRS}" )
message( "" )
message( " gl:" )
message( "   library: ${GL_LIBRARIES}" )
message( "   include: ${GL_INCLUDE_DIRS}" )
message( "" )
message( " glew:" )
message( "   library: ${GLEW_LIBRARIES}" )
message( "   include: ${GLEW_INCLUDE_DIRS}" )
message( "" )
message( " ev:" )
message( "   library: ${EV_LIBRARIES}" )
message( "   include: ${EV_INCLUDE_DIRS}" )
message( "" )
message( " bullet:" )
message( "   library: ${BULLET_LIBRARIES}" )
message( "   include: ${BULLET_INCLUDE_DIRS}" )
message( "" )
