find_package( UnitTest++ REQUIRED )
include_directories (
  ../include
  ${BOOST_INCLUDE_DIRS}
  ${UNITTEST++_INCLUDE_DIR}
  )

add_executable( runTests main.cpp testBoundingBox.cpp testBoundingSphere.cpp)

target_link_libraries( runTests
                      unittest++
                      ${BOOST_LIBRARIES}
                      ${UNITTEST++_LIBRARIES}
                      )
