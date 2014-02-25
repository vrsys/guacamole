include_directories ( ../include ${BOOST_INCLUDE_DIRS} )

add_executable( runTests main.cpp testBoundingBox.cpp )

target_link_libraries( runTests
                      unittest++
                      ${BOOST_LIBRARIES}
                      )
