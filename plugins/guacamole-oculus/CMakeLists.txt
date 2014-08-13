# dependencies
include(find_ovr)

# determine source and header files
file(GLOB_RECURSE GUACAMOLE_OCULUS_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    src/*.cpp
    include/*.h
    include/*.hpp
)

SET(GUACAMOLE_OCULUS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

LINK_DIRECTORIES(${LIB_PATHS} ${OVR_LIBRARY_DIRS})

INCLUDE_DIRECTORIES( ${INCLUDE_PATHS}
                     ${GUACAMOLE_SOURCE_DIR}
                     ${OVR_INCLUDE_DIRS}
                     ${GUACAMOLE_OCULUS_SOURCE_DIR}
)

ADD_LIBRARY( guacamole-oculus SHARED
    ${GUACAMOLE_OCULUS_SRC}
)

ADD_DEPENDENCIES(guacamole-oculus guacamole)

IF (MSVC)
	set_target_properties(guacamole-oculus PROPERTIES COMPILE_FLAGS "-D GUA_OCULUS_LIBRARY")
ENDIF (MSVC)

IF (UNIX)
    LIST(APPEND LIBS udev Xinerama guacamole ${OVR_LIBRARIES})
ELSEIF (MSVC)
    LIST(APPEND LIBS winmm.lib guacamole ${OVR_LIBRARIES})
ENDIF(UNIX)

find_package (Threads)
target_link_libraries( guacamole-oculus debug ${LIBS} ${CMAKE_THREAD_LIBS_INIT} optimized ${LIBS} ${CMAKE_THREAD_LIBS_INIT})

# compile examples

ADD_SUBDIRECTORY(example-oculus)
