## TODO: EXPERIMENTAL - UNSTABLE
find_package(gazebo REQUIRED)

file(GLOB_RECURSE GUA_PAGODA_SRC src/gua/*.cpp)

file(GLOB_RECURSE GUA_PAGODA_INC include/gua/*.hpp)

SOURCE_GROUP("pagoda" FILES ${GUA_PAGODA_SRC})
SOURCE_GROUP("pagoda" FILES ${GUA_PAGODA_INC})

add_library(guacamole-gazebo SHARED ${GUA_PAGODA_INC} ${GUA_PAGODA_SRC})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

list(APPEND GAZEBO_INCLUDE_DIRS "/usr/include/gazebo-7")

include_directories(${GAZEBO_INCLUDE_DIRS})

target_include_directories(guacamole-gazebo PUBLIC ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(guacamole-gazebo PUBLIC guacamole ${GAZEBO_LIBRARIES} pthread)


###############################################################################
# install
###############################################################################

install(TARGETS guacamole-gazebo
        RUNTIME DESTINATION lib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

# header files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/ DESTINATION include/gua FILES_MATCHING PATTERN "*.hpp")

# shader and material resources
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources/ DESTINATION resources
        FILES_MATCHING
        PATTERN "*.frag"
        PATTERN "*.vert"
        PATTERN "*.glsl"
        PATTERN "*.geom"
        PATTERN "*.tctrl"
        PATTERN "*.teval"
        PATTERN "*.obj"
        PATTERN "*.gmd")
