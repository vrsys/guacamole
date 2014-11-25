# dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# determine source and header files
file(GLOB_RECURSE GUACAMOLE_VOLUME_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    src/*.cpp
    include/*.h
    include/*.hpp
)

SET(GUACAMOLE_VOLUME_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-volume includes.")

LINK_DIRECTORIES(${LIB_PATHS} ${AWESOMIUM_LIBRARY_DIRS})

INCLUDE_DIRECTORIES( ${INCLUDE_PATHS}
                     ${GUACAMOLE_SOURCE_DIR}
                     ${AWESOMIUM_INCLUDE_DIR}
                     ${GUACAMOLE_VOLUME_INCLUDE_DIR}
)

ADD_LIBRARY( guacamole-volume SHARED
    ${GUACAMOLE_VOLUME_SRC}
)

ADD_DEPENDENCIES(guacamole-volume guacamole)

IF (MSVC)
	set_target_properties(guacamole-volume PROPERTIES COMPILE_FLAGS "-D GUA_VOLUME_LIBRARY")
ENDIF (MSVC)
