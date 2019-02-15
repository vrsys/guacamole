find_package(gazebo REQUIRED)

FILE(GLOB GUA_NRP_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/gua/nrp/*.cpp)
FILE(GLOB GUA_NRP_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/gua/nrp/*.hpp)

SOURCE_GROUP("nrp" FILES ${GUA_NRP_SRC})
SOURCE_GROUP("nrp" FILES ${GUA_NRP_INC})

SET(GUACAMOLE_NRP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "Path to guacamole-nrp includes.")

find_package(TurboJPEG REQUIRED)
find_package(LZ4 REQUIRED)

add_library(guacamole-nrp SHARED ${GUA_NRP_INC} ${GUA_NRP_SRC})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

list(APPEND GAZEBO_INCLUDE_DIRS "/usr/include/gazebo-7")

target_include_directories(guacamole-nrp PUBLIC ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS} ${GUACAMOLE_NRP_INCLUDE_DIR} ${TurboJPEG_INCLUDE_DIRS} ${LZ4_INCLUDE_DIRS})
target_link_libraries(guacamole-nrp PUBLIC guacamole ${GAZEBO_LIBRARIES} pthread ${TurboJPEG_LIBRARIES} ${LZ4_LIBRARIES})

IF (MSVC)
    set_target_properties(guacamole-nrp PROPERTIES COMPILE_FLAGS "-D GUA_NRP_LIBRARY")
ENDIF (MSVC)

###############################################################################
# install
###############################################################################

IF (MSVC)
    install(TARGETS guacamole-nrp
            CONFIGURATIONS Release
            RUNTIME DESTINATION bin/Release
            LIBRARY DESTINATION lib/Release
            ARCHIVE DESTINATION lib/Release
            )

    install(TARGETS guacamole-nrp
            CONFIGURATIONS Debug
            RUNTIME DESTINATION bin/Debug
            LIBRARY DESTINATION lib/Debug
            ARCHIVE DESTINATION lib/Debug
            )
ELSEIF (UNIX)
    install(TARGETS guacamole-nrp
            RUNTIME DESTINATION lib
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            )
ENDIF (MSVC)

# header files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/ DESTINATION include/gua FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/gua/nrp DESTINATION include/gua/nrp FILES_MATCHING PATTERN "*.hpp")

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
        PATTERN "*.mtl"
        PATTERN "*.gmd"
        PATTERN "*.jpg"
        PATTERN "*.ini")

add_library(nrp_plugin_world SHARED nrp_plugins/GuaWorldPlugin.cpp nrp_plugins/GuaWorldPlugin.hpp)
target_include_directories(nrp_plugin_world PUBLIC ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS} ${GUACAMOLE_NRP_INCLUDE_DIR})
target_link_libraries(nrp_plugin_world PUBLIC guacamole ${GAZEBO_LIBRARIES} pthread)

add_library(nrp_plugin_model SHARED nrp_plugins/GuaInteractiveModelPlugin.cpp nrp_plugins/GuaInteractiveModelPlugin.hpp)
target_include_directories(nrp_plugin_model PUBLIC ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS} ${GUACAMOLE_NRP_INCLUDE_DIR})
target_link_libraries(nrp_plugin_model PUBLIC guacamole ${GAZEBO_LIBRARIES} pthread)

add_library(nrp_plugin_dyngeo SHARED nrp_plugins/GuaDynGeoVisualPlugin.cpp nrp_plugins/GuaDynGeoVisualPlugin.hpp)
target_include_directories(nrp_plugin_dyngeo PUBLIC ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS} ${GUACAMOLE_NRP_INCLUDE_DIR} ${TurboJPEG_INCLUDE_DIRS} ${FASTLZ_INC})
target_link_libraries(nrp_plugin_dyngeo PUBLIC guacamole ${GAZEBO_LIBRARIES} pthread sgtp ${TurboJPEG_LIBRARIES} lz4)