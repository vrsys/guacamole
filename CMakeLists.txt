################################################################
# Project GUACAMOLE
################################################################

PROJECT(GUACAMOLE CXX)

# neccessary for try_compile_scripts under linux
ENABLE_LANGUAGE(C)

# version number
set(GUACAMOLE_MAJOR 1)
set(GUACAMOLE_MINOR 8)
set(GUACAMOLE_PATCH 18)
set(GUACAMOLE_VERSION ${GUACAMOLE_MAJOR}.${GUACAMOLE_MINOR}.${GUACAMOLE_PATCH})
set(GUACAMOLE_DESCRIPTION "GUACAMOLE - an astonishing virtual reality engine")
set(GUACAMOLE_HOMEPAGE "http://www.GUACAMOLE.org")
set(GUACAMOLE_EXENAME "GUACAMOLE")
set(GUACAMOLE_PACKAGENAME "GUACAMOLE")

option (GUACAMOLE_ENABLE_PHYSICS "Set to enable physics support." OFF)
option (GUACAMOLE_GLFW3 "Set to enable glfw3 support." ON)
option (GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES "Set to enable profile using per-pass time queries." OFF)
option (GUACAMOLE_RUNTIME_PROGRAM_COMPILATION "Set to enable to runtime generation of ubershaders." ON)
option (GUACAMOLE_ENABLE_NVIDIA_3D_VISION "Set to enable NVIDIA 3D Vision active stereo." ON)
option (GUACAMOLE_FBX "Set to enable FBX support." ON)

# We require at least version 3.0.2
cmake_minimum_required(VERSION 3.0.2)

if (UNIX)
  find_package(PkgConfig)
endif(UNIX)

# Location where cmake first looks for modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        SET(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Default install
                 prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)


if(GUACAMOLE_ENABLE_NVIDIA_3D_VISION)
  set(GUACAMOLE_NVIDIA_3D_VISION_FIRMWARE_PATH "${CMAKE_INSTALL_PREFIX}/lib_external/nvstusb.fw")
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/externals/nvstusb/ DESTINATION lib_external
          FILES_MATCHING
          PATTERN "*.fw"
         )
endif(GUACAMOLE_ENABLE_NVIDIA_3D_VISION)
###############################################################################
# write Config.h
###############################################################################

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/config.hpp.in
  ${PROJECT_BINARY_DIR}/include/gua/config.hpp
  )

install(FILES ${PROJECT_BINARY_DIR}/include/gua/config.hpp DESTINATION include/gua)

# add the binary tree to the search path for include files
# so that we will find the generated config.hpp
include_directories("${PROJECT_BINARY_DIR}/include")

################################################################
# Configure and find libraries
################################################################
set(GLOBAL_EXT_DIR ${GUACAMOLE_SOURCE_DIR}/externals)

include(define_macros)
include(find_compiler)

if (UNIX)
  pkg_check_modules(ASSIMP REQUIRED assimp)
  pkg_check_modules(GL REQUIRED gl)
  pkg_check_modules(GLEW REQUIRED glew)
  pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
  include(find_x11)
elseif (WIN32)
  include(FindAssimp)
  include(FindFreeImage)
  set(GL_LIBRARIES OpenGL32.lib)
endif (UNIX)

include(FindSchism)

if (UNIX)
  set (BOOST_ROOT "/opt/boost/latest" CACHE PATH "Set to your boost install")
  find_package(Boost COMPONENTS
    system
    regex
    chrono
    thread
    filesystem
    REQUIRED
    )
elseif (WIN32)
  include(FindBoostWindows)
endif (UNIX)

include(FindZMQ)

if (GUACAMOLE_ENABLE_PHYSICS)
  if (UNIX)
    set (BULLET_ROOT "/opt/bullet/default" CACHE PATH "Set to your bullet install path.")
  elseif (WIN32)
    set (BULLET_ROOT "externals/" CACHE PATH "Set to your bullet install path.")
  endif (UNIX)
  find_package(Bullet REQUIRED)
endif (GUACAMOLE_ENABLE_PHYSICS)

if (GUACAMOLE_FBX)
  include(FindFBX)
  set (FBX_ROOT "externals/fbx" CACHE PATH "Set to your FBX SDK install path.")
  # set (FBX_INCLUDE_DIRS "externals/fbx/include" CACHE PATH "Set to your FBX SDK install path.")
  # if (UNIX)
  #   set (FBXSDK_ROOT "/opt/bullet/default" CACHE PATH "Set to your bullet install path.")
  # elseif (WIN32)
  #   set (FBXSDK_ROOT "externals/" CACHE PATH "Set to your bullet install path.")
  # endif (UNIX)
  find_package(FBX REQUIRED)
endif (GUACAMOLE_FBX)

if (GUACAMOLE_GLFW3)
  find_package(GLFW3 REQUIRED)
endif (GUACAMOLE_GLFW3)

find_package(Json REQUIRED)

set(LIB_PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
    ${ASSIMP_LIBRARY_DIRS}
    ${JSON_LIBRARY_DIRS}
    ${GL_LIBRARY_DIRS}
    ${GLEW_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
    ${BULLET_LIBRARY_DIRS}
    ${FBX_LIBRARY_DIRS}
    ${SCHISM_LIBRARY_DIRS}
    ${ZMQ_LIBRARY_DIRS}
    ${X11_LIBRARY_DIRS}
)

set(INCLUDE_PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/build
    ${CMAKE_CURRENT_SOURCE_DIR}/externals
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/guacamole-gui/include
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/guacamole-plod/include
    ${CMAKE_CURRENT_SOURCE_DIR}/plugins/guacamole-volume/include
    ${SCHISM_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIR}
    ${JSON_INCLUDE_DIRS}
    ${GL_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${BULLET_INCLUDE_DIRS}
    ${FBX_INCLUDE_DIRS}
    ${ZMQ_INCLUDE_DIRS}
    ${X11_INCLUDE_DIRS}
)

if (GUACAMOLE_GLFW3)
  LIST(APPEND INCLUDE_PATHS ${GLFW_INCLUDE_DIRS})
endif (GUACAMOLE_GLFW3)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

IF (UNIX)
	set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -s -O4 --std=c++11")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO  "-O2 -g -DNDEBUG --std=c++11")
	set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall --std=c++11")
ELSEIF(MSVC)
	set(CMAKE_CXX_FLAGS_RELEASE "-D NDEBUG -D _USE_MATH_DEFINES -D _SECURE_SCL=0 -D _SCL_SECURE_NO_WARNINGS -D _CRT_SECURE_NO_DEPRECATE /MD /MP")
	set(CMAKE_CXX_FLAGS_DEBUG "-D_DEBUG -D _USE_MATH_DEFINES /MDd /Zi /MP")
ENDIF(UNIX)

################################################################
# Create GuaRC
################################################################
IF (NOT ${GUACAMOLE_RUNTIME_PROGRAM_COMPILATION})
  add_subdirectory(guarc)
ENDIF (NOT ${GUACAMOLE_RUNTIME_PROGRAM_COMPILATION})

################################################################
# Generate RC data
################################################################

IF (NOT ${GUACAMOLE_RUNTIME_PROGRAM_COMPILATION})

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


  IF (UNIX)
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

  ELSEIF(MSVC)
      ADD_CUSTOM_TARGET(CompileResources ALL
         DEPENDS ${RESOURCES} guarc
      )

      SET (_GUARC_EXECUTABLE "")
      SET (_GUARC_EXECUTABLE "/guarc/$(Configuration)/guarc.exe")

      ADD_CUSTOM_COMMAND(TARGET CompileResources POST_BUILD
    	  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}${_GUARC_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/resources
          ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/generated/R.inl
          ${CMAKE_CURRENT_SOURCE_DIR}/src/gua/generated/R.inl
      )
  ENDIF(UNIX)

ENDIF (NOT ${GUACAMOLE_RUNTIME_PROGRAM_COMPILATION})

################################################################
# Build external sources
################################################################

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
add_subdirectory(externals)

################################################################
# Create libraries
################################################################

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
add_subdirectory(src)

################################################################
# Create plugins
################################################################

add_subdirectory(plugins)

################################################################
# Testing
################################################################

option (GUACAMOLE_TESTS "Enable testing." OFF)

if (GUACAMOLE_TESTS)
  add_subdirectory(tests)
  enable_testing()
  add_test( NAME testGUA COMMAND runTests )
endif (GUACAMOLE_TESTS)

################################################################
# Examples
################################################################

option (GUACAMOLE_EXAMPLES "Build examples." ON)

if (GUACAMOLE_EXAMPLES)
  add_subdirectory(examples)
endif (GUACAMOLE_EXAMPLES)

################################################################
# Summary
################################################################

message(STATUS "" )
message(STATUS "Summary:" )
message(STATUS " build type: ${CMAKE_BUILD_TYPE}" )
message(STATUS "" )
message(STATUS " schism:" )
message(STATUS "   library: ${SCHISM_LIBRARIES}" )
message(STATUS "   library path: ${SCHISM_LIBRARY_DIRS}" )
message(STATUS "   include: ${SCHISM_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " boost:" )
message(STATUS "   library: ${Boost_LIBRARIES}" )
message(STATUS "   library path: ${Boost_LIBRARY_DIRS}" )
message(STATUS "   include: ${Boost_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " assimp:" )
message(STATUS "   library: ${ASSIMP_LIBRARIES}" )
message(STATUS "   library path: ${ASSIMP_LIBRARY_DIRS}" )
message(STATUS "   include: ${ASSIMP_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " json:" )
message(STATUS "   library: ${JSON_LIBRARIES}" )
message(STATUS "   library path: ${JSON_LIBRARY_DIRS}" )
message(STATUS "   include: ${JSON_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " x11:" )
message(STATUS "   library: ${X11_LIBRARIES}" )
message(STATUS "   library path: ${X11_LIBRARY_DIRS}" )
message(STATUS "   include: ${X11_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " gl:" )
message(STATUS "   library: ${GL_LIBRARIES}" )
message(STATUS "   include: ${GL_INCLUDE_DIRS}" )
message(STATUS "" )
message(STATUS " glew:" )
message(STATUS "   library: ${GLEW_LIBRARIES}" )
message(STATUS "   include: ${GLEW_INCLUDE_DIRS}" )
message(STATUS "" )

if (GUACAMOLE_ENABLE_PHYSICS)
  message(STATUS " bullet:" )
  message(STATUS "   library: ${BULLET_LIBRARIES}" )
  message(STATUS "   library path: ${BULLET_LIBRARY_DIRS}" )
  message(STATUS "   include: ${BULLET_INCLUDE_DIRS}" )
  message(STATUS "" )
endif (GUACAMOLE_ENABLE_PHYSICS)


if (GUACAMOLE_FBX)
  message(STATUS " FBX SDK:" )
  message(STATUS "   library: ${FBX_LIBRARIES}" )
  message(STATUS "   library path: ${FBX_LIBRARY_DIRS}" )
  message(STATUS "   include: ${FBX_INCLUDE_DIRS}" )
  message(STATUS "" )
endif (GUACAMOLE_FBX)

if (GUACAMOLE_GLFW3)
  message(STATUS " glfw:" )
  message(STATUS "   library: ${GLFW_LIBRARIES}" )
  message(STATUS "   library path: ${GLFW_LIBRARY_DIRS}" )
  message(STATUS "   include: ${GLFW_INCLUDE_DIRS}" )
  message(STATUS "" )
endif (GUACAMOLE_GLFW3)

message(STATUS "" )