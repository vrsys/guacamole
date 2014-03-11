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

#include <iostream>

namespace gua {

class Logger {

 public:

  static bool enable_debug;
  static bool enable_message;
  static bool enable_warning;
  static bool enable_error;

  #define LOG_DEBUG   debug_impl  (__FILE__, __LINE__)
  #define LOG_MESSAGE message_impl(__FILE__, __LINE__)
  #define LOG_WARNING warning_impl(__FILE__, __LINE__)
  #define LOG_ERROR   error_impl  (__FILE__, __LINE__)

  static std::ostream& debug_impl(const char* file, int line);
  static std::ostream& message_impl(const char* file, int line);
  static std::ostream& warning_impl(const char* file, int line);
  static std::ostream& error_impl(const char* file, int line);
};

}

#endif  // GUA_LOGGER_HPP
