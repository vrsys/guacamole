/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/
#ifndef GUA_PLATFORM_HPP
#define GUA_PLATFORM_HPP

#include <boost/preprocessor.hpp>
#include <boost/preprocessor/detail/is_nullary.hpp>

// GUA_PLATFORM will be set to one of these
#define GUA_PLATFORM_WINDOWS 1
#define GUA_PLATFORM_LINUX 2
#define GUA_PLATFORM_APPLE 3

// GUA_COMPILER will be set to one of these
#define GUA_COMPILER_MSVC 1
#define GUA_COMPILER_GNUC 2

// GUA_ARCHITECTURE_TYPE will be set to one of these
#define GUA_ARCHITECTURE_32 1
#define GUA_ARCHITECTURE_64 2

// compiler
#if defined(_MSC_VER)
#define GUA_COMPILER GUA_COMPILER_MSVC
#define GUA_COMPILER_VER _MSC_VER
#define gua_force_inline __force_inline
#define gua_align(border) __declspec(align(border))

#elif defined(__GNUC__)
#define GUA_COMPILER GUA_COMPILER_GNUC
#define GUA_COMPILER_VER (((__GNUC__)*100) + (__GNUC_MINOR__ * 10) + __GNUC_PATCHLEVEL__)
#define gua_force_inline __attribute__((always_inline))
#define gua_align(border) __attribute__((aligned(border)))
#else
#error "unknown compiler"
#endif

// platform
#if defined(__WIN32__) || defined(_WIN32) || defined(_WIN64)
#define GUA_PLATFORM GUA_PLATFORM_WINDOWS
#elif defined(__APPLE_CC__)
#define GUA_PLATFORM GUA_PLATFORM_APPLE
#else
#define GUA_PLATFORM GUA_PLATFORM_LINUX
#endif

// architecture
#if defined(__x86_64__) || defined(_M_X64)
#define GUA_ARCHITECTURE_TYPE GUA_ARCHITECTURE_64
#else
#define GUA_ARCHITECTURE_TYPE GUA_ARCHITECTURE_32
#endif

// compiler messages
#define TO_STR(x) BOOST_PP_STRINGIZE(x)
#define todo(msg)                                                                                                                                                                                      \
    message(__FILE__ "(" TO_STR(__LINE__) "): "                                                                                                                                                        \
                                          "todo: " #msg)
#define fix_me(msg)                                                                                                                                                                                    \
    message(__FILE__ "(" TO_STR(__LINE__) "): "                                                                                                                                                        \
                                          "fix_me: " #msg)
#define warn_message(msg)                                                                                                                                                                              \
    message(__FILE__ "(" TO_STR(__LINE__) "): "                                                                                                                                                        \
                                          "warning: " #msg)

// windows related
#ifndef GUA_STATIC_BUILD
#if GUA_PLATFORM == GUA_PLATFORM_WINDOWS
#if GUA_COMPILER == GUA_COMPILER_MSVC
#pragma warning(disable : 4251) // needs to have dll-interface to be used by clients of class
#pragma warning(disable : 4275) // non dll-interface class used as base for dll-interface class

#if defined(GUA_LIBRARY)
#define GUA_DLL __declspec(dllexport)
#else
#define GUA_DLL __declspec(dllimport)
#endif
#endif
#endif
#else
#define GUA_DLL
#endif

// Linux, Apple
#if GUA_PLATFORM == GUA_PLATFORM_LINUX || GUA_PLATFORM == GUA_PLATFORM_APPLE
#define GUA_DLL
#endif

#if GUA_PLATFORM == GUA_PLATFORM_WINDOWS
#ifndef NDEBUG
#define GUA_DEBUG 1
#else
#define GUA_DEBUG 0
#endif
#endif

// Linux, Apple
#if GUA_PLATFORM == GUA_PLATFORM_LINUX || GUA_PLATFORM == GUA_PLATFORM_APPLE
/*#ifndef NDEBUG
  #define GUA_DEBUG 1
#else
  #define GUA_DEBUG 0
#endif
*/
#define GUA_DEBUG 0
#endif

#endif // namespace GUA_PLATFORM_HPP
