##############################################################################
# search paths
##############################################################################
SET(ASSIMP_SEARCH_PATHS
	${GLOBAL_EXT_DIR}/assimp/include
	/opt/assimp/current
	/usr/include
	${GLOBAL_EXT_DIR}/assimp/lib
	/opt/assimp/current/assimp-build
	/usr/lib
)

##############################################################################
# search
##############################################################################
find_path (ASSIMP_INCLUDE_DIR
           NAMES assimp/ai_assert.h
		   HINTS
		   NO_DEFAULT_PATH
		   NO_CMAKE_ENVIRONMENT_PATH
		   NO_CMAKE_SYSTEM_PATH
		   NO_SYSTEM_ENVIRONMENT_PATH
		   NO_CMAKE_PATH
		   CMAKE_FIND_FRAMEWORK NEVER
		   PATHS
		   ${ASSIMP_SEARCH_PATHS}
)

find_library (ASSIMP_LIBRARY_DEBUG 
              NAMES assimpd libassimpd libassimp_d assimp-${COMPILER_SUFFIX}-mtd
			  PATH_SUFFIXES release debug 
			  PATHS ${ASSIMP_SEARCH_PATHS})
			  
find_library (ASSIMP_LIBRARY_RELEASE
              NAMES assimp libassimp assimp-${COMPILER_SUFFIX}-mt 
			  PATH_SUFFIXES release debug 
			  PATHS ${ASSIMP_SEARCH_PATHS})

if (ASSIMP_INCLUDE_DIR AND ASSIMP_LIBRARY_RELEASE)
set(ASSIMP_FOUND TRUE)
endif()
if (ASSIMP_LIBRARY_RELEASE)
set (ASSIMP_LIBRARY ${ASSIMP_LIBRARY_RELEASE})
endif()
if (ASSIMP_LIBRARY_DEBUG AND ASSIMP_LIBRARY_RELEASE)
set (ASSIMP_LIBRARY debug ${ASSIMP_LIBRARY_DEBUG} optimized ${ASSIMP_LIBRARY_RELEASE} )
endif()
if (ASSIMP_FOUND)
MESSAGE(STATUS "-- Found Assimp ${ASSIMP_LIBRARIES}")
mark_as_advanced (ASSIMP_INCLUDE_DIR ASSIMP_LIBRARY ASSIMP_LIBRARIES)
endif()