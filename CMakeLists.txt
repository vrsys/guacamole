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

if (UNIX)
	find_package(PkgConfig)
endif(UNIX)

# Location where cmake first looks for modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

################################################################
# Configure and find libraries
################################################################
set(GLOBAL_EXT_DIR ${GUACAMOLE_SOURCE_DIR}/externals)

if (UNIX)
	pkg_check_modules(ASSIMP REQUIRED assimp)
	pkg_check_modules(JSON REQUIRED jsoncpp)
	pkg_check_modules(GL REQUIRED gl)
	pkg_check_modules(GLEW REQUIRED glew)
elseif (WIN32)
	include(find_assimp)	
endif (UNIX)

include(find_compiler)
include(find_schism)
include(find_boost)
include(find_cuda)
#include(find_ev)
include(find_bullet)
include(find_json)

set(LIBS
    ${SCHISM_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${JSON_LIBRARIES}
    ${GL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${EV_LIBRARIES}
    ${BULLET_LIBRARIES}
	${CUDA_LIBRARIES}
    #profiler
    #tcmalloc
)

set(LIB_PATHS
    ${ASSIMP_LIBRARY_DIRS}
    ${JSON_LIBRARY_DIRS}
    ${GL_LIBRARY_DIRS}
    ${GLEW_LIBRARY_DIRS}
	${BOOST_LIBRARY_DIRS}
	${BULLET_LIBRARY_DIRS}
	${SCHISM_LIBRARY_DIRS}
	${CUDA_LIBRARY_DIRS}
)

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
	${CUDA_INCLUDE_DIRS}
)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

IF (UNIX)
	set(CMAKE_CXX_FLAGS_RELEASE "-s -O4 --std=c++0x -fpermissive")
	set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall --std=c++0x -fpermissive")
ELSEIF(MSVC)
	set(CMAKE_CXX_FLAGS_RELEASE "-D _SECURE_SCL=0 -D _SCL_SECURE_NO_WARNINGS -D _CRT_SECURE_NO_DEPRECATE /MD /MP")
	set(CMAKE_CXX_FLAGS_DEBUG "-D_DEBUG /MDd /Zi")
ENDIF(UNIX)

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

MESSAGE(${RESOURCES})

if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/generated/)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/generated)
endif()

if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated)
endif()

# ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/R.inl
	# COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/guarc/guarc
    # ${CMAKE_CURRENT_SOURCE_DIR}/resources
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/generated/R.inl
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/R.inl
	# DEPENDS ${RESOURCES} guarc
# )

ADD_CUSTOM_TARGET(CompileResources ALL
   DEPENDS ${RESOURCES} guarc
)

SET (_GUARC_EXECUTABLE "")
IF (UNIX)
	SET (_GUARC_EXECUTABLE "/guarc/guarc")
ELSEIF(WIN32)
	SET (_GUARC_EXECUTABLE "/guarc/$(Configuration)/guarc.exe")
ENDIF(UNIX)

ADD_CUSTOM_COMMAND(TARGET CompileResources POST_BUILD
	COMMAND ${CMAKE_CURRENT_SOURCE_DIR}${_GUARC_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/resources
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/generated/R.inl
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/R.inl
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
message( "   library path: ${SCHISM_LIBRARY_DIRS}" )
message( "   include: ${SCHISM_INCLUDE_DIRS}" )
message( "" )
message( " boost:" )
message( "   library: ${BOOST_LIBRARIES}" )
message( "   library path: ${BOOST_LIBRARY_DIRS}" )
message( "   include: ${BOOST_INCLUDE_DIRS}" )
message( "" )
message( " assimp:" )
message( "   library: ${ASSIMP_LIBRARIES}" )
message( "   library path: ${ASSIMP_LIBRARY_DIRS}" )
message( "   include: ${ASSIMP_INCLUDE_DIRS}" )
message( "" )
message( " json:" )
message( "   library: ${JSON_LIBRARIES}" )
message( "   library path: ${JSON_LIBRARY_DIRS}" )
message( "   include: ${JSON_INCLUDE_DIRS}" )
message( "" )
message( " cuda:" )
message( "   library: ${CUDA_LIBRARIES}" )
message( "   library path: ${CUDA_LIBRARY_DIRS}" )
message( "   include: ${CUDA_INCLUDE_DIRS}" )
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
message( "   library path: ${BULLET_LIBRARY_DIRS}" )
message( "   include: ${BULLET_INCLUDE_DIRS}" )
message( "" )
