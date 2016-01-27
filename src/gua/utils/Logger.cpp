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

// class header
#include <gua/utils/Logger.hpp>
#include <scm/core/log/console_color.h>
#include <sstream>
#include <boost/iostreams/stream.hpp>

#define PRINT_RED 0
#define PRINT_GREEN 2
#define PRINT_YELLOW 3
#define PRINT_BLUE 4
#define PRINT_PURPLE 5
#define PRINT_TURQUOISE 6

#define PRINT_RED_BOLD 7
#define PRINT_GREEN_BOLD 8
#define PRINT_YELLOW_BOLD 9
#define PRINT_BLUE_BOLD 10
#define PRINT_PURPLE_BOLD 11
#define PRINT_TURQUOISE_BOLD 12

namespace gua {

bool Logger::enable_debug = true;
bool Logger::enable_message = true;
bool Logger::enable_warning = true;
bool Logger::enable_error = true;

namespace {
  namespace io = boost::iostreams;
  io::null_sink                     null_sink;
  io::stream_buffer<io::null_sink>  null_buffer(null_sink);
  std::ostream                      dev_null(&null_buffer);

  std::string location_string(const char* f, int l) {
    #ifdef _WIN32
      std::string file(std::string(f).substr(std::string(f).find_last_of('\\') + 1).c_str());
    #else
      std::string file(std::string(f).substr(std::string(f).find_last_of('/') + 1).c_str());
    #endif
    std::stringstream sstr;
    sstr << "[" << file << ":" << l << "]";
    return sstr.str();
  }

  void set_color(int color) {
    #ifdef _WIN32
      switch (color) {
      case PRINT_RED: scm::util::fg_red(std::cout); break;
      case PRINT_GREEN: scm::util::fg_green(std::cout); break;
      case PRINT_YELLOW: scm::util::fg_yellow(std::cout); break;
      case PRINT_BLUE: scm::util::fg_blue(std::cout); break;
      case PRINT_PURPLE: scm::util::fg_magenta(std::cout); break;
      case PRINT_TURQUOISE: scm::util::fg_cyan(std::cout); break;

      case PRINT_RED_BOLD: scm::util::fg_dk_red(std::cout); break;
      case PRINT_GREEN_BOLD: scm::util::fg_dk_green(std::cout); break;
      case PRINT_YELLOW_BOLD: scm::util::fg_dk_yellow(std::cout); break;
      case PRINT_BLUE_BOLD:scm::util::fg_dk_blue(std::cout); break;
      case PRINT_PURPLE_BOLD: scm::util::fg_dk_magenta(std::cout); break;
      case PRINT_TURQUOISE_BOLD: scm::util::fg_dk_cyan(std::cout); break;
      }
    #else
      switch(color) {
        case PRINT_RED: std::cout << "\x001b[0;31m"; break;
        case PRINT_GREEN: std::cout << "\x001b[0;32m"; break;
        case PRINT_YELLOW: std::cout << "\x001b[0;33m"; break;
        case PRINT_BLUE: std::cout << "\x001b[0;34m"; break;
        case PRINT_PURPLE: std::cout << "\x001b[0;35m"; break;
        case PRINT_TURQUOISE: std::cout << "\x001b[0;36m"; break;
        
        case PRINT_RED_BOLD: std::cout << "\x001b[1;31m"; break;
        case PRINT_GREEN_BOLD: std::cout << "\x001b[1;32m"; break;
        case PRINT_YELLOW_BOLD: std::cout << "\x001b[1;33m"; break;
        case PRINT_BLUE_BOLD: std::cout << "\x001b[1;34m"; break;
        case PRINT_PURPLE_BOLD: std::cout << "\x001b[1;35m"; break;
        case PRINT_TURQUOISE_BOLD: std::cout << "\x001b[1;36m"; break;
      }
    #endif
  }

  void reset_color() {
    #ifdef _WIN32
      scm::util::reset_color(std::cout);
    #else
      std::cout << "\x001b[0m";
    #endif
  }
}

////////////////////////////////////////////////////////////////////////////////

std::ostream& Logger::debug_impl(const char* file, int line) {
    if (enable_debug) {
        set_color(PRINT_BLUE);
        std::cout << "[GUA][D]" << location_string(file, line);
        reset_color();
        return std::cout << " ";
    }
    
    return dev_null;
}

////////////////////////////////////////////////////////////////////////////////

std::ostream& Logger::message_impl(const char* file, int line) {
    if (enable_message) {
        set_color(PRINT_GREEN);
        std::cout << "[GUA][M]" << location_string(file, line);
        reset_color();
        return std::cout << " ";
    }

    return dev_null;
}

////////////////////////////////////////////////////////////////////////////////

std::ostream& Logger::warning_impl(const char* file, int line) {
    if (enable_warning) {
        set_color(PRINT_YELLOW);
        std::cout << "[GUA][W]" << location_string(file, line);
        reset_color();
        return std::cout << " ";
    }

    return dev_null;
}

////////////////////////////////////////////////////////////////////////////////

std::ostream& Logger::error_impl(const char* file, int line) {
    if (enable_error) {
        set_color(PRINT_RED);
        std::cout << "[GUA][E]" << location_string(file, line);
        reset_color();
        return std::cout << " ";
    }

    return dev_null;
}

////////////////////////////////////////////////////////////////////////////////

}
