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

#ifndef GUA_LOGGER_HPP
#define GUA_LOGGER_HPP

#include <cstring>
#include <cstdio>

#define PRINT_RED "\x001b[0;31m"
#define PRINT_GREEN "\x001b[0;32m"
#define PRINT_YELLOW "\x001b[0;33m"
#define PRINT_BLUE "\x001b[0;34m"
#define PRINT_PURPLE "\x001b[0;35m"
#define PRINT_TURQUOISE "\x001b[0;36m"

#define PRINT_RED_BOLD "\x001b[1;31m"
#define PRINT_GREEN_BOLD "\x001b[1;32m"
#define PRINT_YELLOW_BOLD "\x001b[1;33m"
#define PRINT_BLUE_BOLD "\x001b[1;34m"
#define PRINT_PURPLE_BOLD "\x001b[1;35m"
#define PRINT_TURQUOISE_BOLD "\x001b[1;36m"

#define PRINT_RESET "\x001b[0m"

#define GETFILENAME(_path) \
  std::string(_path).substr(std::string(_path).find_last_of('/') + 1).c_str()

#define WHERE_STR "[%s:%d] "
#define WHERE_ARG GETFILENAME(__FILE__), __LINE__

#define MESSAGE(_fmt, ...)                                        \
  printf(PRINT_GREEN "[GUA][M]" WHERE_STR PRINT_RESET _fmt "\n", \
         WHERE_ARG,                                               \
         ##__VA_ARGS__)
#ifndef MESSAGE
#define MESSAGE(...)
#endif

#define WARNING(_fmt, ...)                                         \
  printf(PRINT_YELLOW "[GUA][W]" WHERE_STR PRINT_RESET _fmt "\n", \
         WHERE_ARG,                                                \
         ##__VA_ARGS__)
#ifndef WARNING
#define WARNING(...)
#endif

#define ERROR(_fmt, ...)                                        \
  printf(PRINT_RED "[GUA][E]" WHERE_STR PRINT_RESET _fmt "\n", \
         WHERE_ARG,                                             \
         ##__VA_ARGS__)
#ifndef ERROR
#define ERROR(...)
#endif

#define DEBUG(_fmt, ...)                                         \
  printf(PRINT_BLUE "[GUA][D]" WHERE_STR PRINT_RESET _fmt "\n", \
         WHERE_ARG,                                              \
         ##__VA_ARGS__)
#ifndef DEBUG
#define DEBUG(...)
#endif

#define PROFILING(_fmt, ...) \
  printf(PRINT_PURPLE "[GUA][P]" PRINT_RESET _fmt "\n", ##__VA_ARGS__)
#ifndef PROFILING
#define PROFILING(...)
#endif

#endif  // GUA_LOGGER_HPP
