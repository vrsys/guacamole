if (WIN32)
    if (MSVC71)
        set (_COMPILER_SUFFIX "vc71")
        set (_COMPILER_SUFFIX_VERSION "71")
    endif(MSVC71)

    if (MSVC80)
        set (_COMPILER_SUFFIX "vc80")
        set (_COMPILER_SUFFIX_VERSION "vc80")
    endif(MSVC80)

    if (MSVC90)
        set (_COMPILER_SUFFIX "vc90")
        set (_COMPILER_SUFFIX_VERSION "90")
    endif(MSVC90)

    if (MSVC10)
        set (_COMPILER_SUFFIX "vc100")
        set (_COMPILER_SUFFIX_VERSION "100")
    endif(MSVC10)

    if (MSVC11)
        set (_COMPILER_SUFFIX "vc110")
        set (_COMPILER_SUFFIX_VERSION "110")
    endif(MSVC11)
	
	if (MSVC12)
        set (_COMPILER_SUFFIX "vc120")
        set (_COMPILER_SUFFIX_VERSION "120")
    endif(MSVC12)

    if (MSVC14)
        set (_COMPILER_SUFFIX "vc141")
        set (_COMPILER_SUFFIX_VERSION "141")
    endif(MSVC14)

    if (MSVC15)
        set (_COMPILER_SUFFIX "vc141")
        set (_COMPILER_SUFFIX_VERSION "141")
    endif(MSVC15)
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
        string(REGEX REPLACE ".* ([0-9])\\.([0-9])\\.[0-9].*" "\\1\\2" _COMPILER_VERSION ${_COMPILER_VERSION})
        set (_COMPILER_SUFFIX "gcc${_COMPILER_VERSION}")
    endif (CMAKE_COMPILER_IS_GNUCXX)
endif(UNIX)

if (_COMPILER_SUFFIX STREQUAL "")
    message(FATAL_ERROR "find_compiler.cmake: unable to identify supported compiler")
else (_COMPILER_SUFFIX STREQUAL "")
    set(COMPILER_SUFFIX       ${_COMPILER_SUFFIX}       CACHE STRING "The boost style compiler suffix")
    set(COMPILER_SUFFIX_VERSION       ${_COMPILER_SUFFIX_VERSION}       CACHE STRING "The compiler suffix version only")
endif (_COMPILER_SUFFIX STREQUAL "")
