find_package( UnitTest++ REQUIRED )
include_directories (
  ../include
  ${Boost_INCLUDE_DIRS}
  ${UNITTEST++_INCLUDE_DIR}
  )

add_executable( runTests main.cpp testBoundingBox.cpp testBoundingSphere.cpp)

target_link_libraries( runTests
                      unittest++
                      ${Boost_LIBRARIES}
                      ${UNITTEST++_LIBRARIES}
                      )
