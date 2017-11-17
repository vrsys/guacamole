## TODO: EXPERIMENTAL - UNSTABLE
find_package(gazebo REQUIRED)

add_library(guacamole-gazebo INTERFACE)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

list(APPEND GAZEBO_INCLUDE_DIRS "/usr/include/gazebo-7")

include_directories(${GAZEBO_INCLUDE_DIRS})

target_include_directories(guacamole-gazebo INTERFACE ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(guacamole-gazebo INTERFACE guacamole ${GAZEBO_LIBRARIES} pthread)


###############################################################################
# install
###############################################################################

install(TARGETS guacamole-gazebo
        RUNTIME DESTINATION lib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
