if (WIN32)
    if (MSVC71)
        set (COMPILER_SUFFIX "vc71")
    endif(MSVC71)

    if (MSVC80)
        set (COMPILER_SUFFIX "vc80")
    endif(MSVC80)

    if (MSVC90)
        set (COMPILER_SUFFIX "vc90")
    endif(MSVC90)

    if (MSVC10)
        set (COMPILER_SUFFIX "vc100")
    endif(MSVC10)

    if (MSVC11)
        set (COMPILER_SUFFIX "vc110")
    endif(MSVC11)
	
	if (MSVC12)
        set (COMPILER_SUFFIX "vc120")
    endif(MSVC12)
endif (WIN32)

macro(GET_WIN32_WINNT version)
    if (WIN32 AND CMAKE_SYSTEM_VERSION)
        set(ver ${CMAKE_SYSTEM_VERSION})
        string(REPLACE "." "" ver ${ver})
        string(REGEX REPLACE "([0-9])" "0\\1" ver ${ver})
        set(${version} "0x${ver}")
    endif()
endmacro(GET_WIN32_WINNT)

if (UNIX)
    if (CMAKE_COMPILER_IS_GNUCXX)
        #find out the version of gcc being used.
        exec_program(${CMAKE_CXX_COMPILER}
                     ARGS --version
                     OUTPUT_VARIABLE _COMPILER_VERSION
        )
        message(${_COMPILER_VERSION})
        string(REGEX REPLACE ".* ([0-9])\\.([0-9])\\.[0-9].*" "\\1\\2" _COMPILER_VERSION ${_COMPILER_VERSION})
        message(${_COMPILER_VERSION})
        set (GUA_COMPILER_SUFFIX "gcc${_COMPILER_VERSION}")
        #set (GUA_COMPILER_SUFFIX "")
        message(${GUA_COMPILER_SUFFIX})
    endif (CMAKE_COMPILER_IS_GNUCXX)
endif(UNIX)

if (GUA_COMPILER_SUFFIX STREQUAL "")
    message(FATAL_ERROR "schism_compiler.cmake: unable to identify supported compiler")
else (GUA_COMPILER_SUFFIX STREQUAL "")
    set(GUA_COMPILER_SUFFIX       ${GUA_COMPILER_SUFFIX}       CACHE STRING "The boost style compiler suffix")
endif (GUA_COMPILER_SUFFIX STREQUAL "")
