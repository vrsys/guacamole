# dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(find_awesomium)

# determine source and header files
file(GLOB_RECURSE GUACAMOLE_GUI_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    src/*.cpp
    include/*.h
    include/*.hpp
)

SET(GUACAMOLE_GUI_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-gui includes.")

LINK_DIRECTORIES(${LIB_PATHS} ${OVR_LIBRARY_DIRS})

INCLUDE_DIRECTORIES( ${INCLUDE_PATHS}
                     ${GUACAMOLE_SOURCE_DIR}
                     ${AWESOMIUM_INCLUDE_DIR}
                     ${GUACAMOLE_GUI_INCLUDE_DIR}
)

ADD_LIBRARY( guacamole-gui SHARED
    ${GUACAMOLE_GUI_SRC}
)

ADD_DEPENDENCIES(guacamole-gui guacamole)

IF (MSVC)
	set_target_properties(guacamole-gui PROPERTIES COMPILE_FLAGS "-D GUA_GUI_LIBRARY")
ENDIF (MSVC)

IF (UNIX)
    LIST(APPEND LIBS guacamole ${AWESOMIUM_LIBRARIES})
ELSEIF (MSVC)
    LIST(APPEND LIBS guacamole ${AWESOMIUM_LIBRARIES})
ENDIF(UNIX)


target_link_libraries( guacamole-gui debug ${LIBS} optimized ${LIBS})
