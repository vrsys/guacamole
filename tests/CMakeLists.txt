find_package( UnitTest++ REQUIRED )
include_directories (
  ../include
  ${Boost_INCLUDE_DIRS}
  ${UNITTEST++_INCLUDE_DIR}
  )

add_executable( runTests main.cpp testBoundingBox.cpp testBoundingSphere.cpp)

IF (UNIX)
  target_link_libraries( runTests
                        general ${UNITTEST++_LIBRARY}
                        )
ELSEIF (MSVC)
  target_link_libraries( runTests
                        optimized ${UNITTEST++_LIBRARY} debug ${UNITTEST++_LIBRARY_DEBUG}
                        )
ENDIF()