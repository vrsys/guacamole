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

#ifndef GUA_GUI_PATHS_HPP
#define GUA_GUI_PATHS_HPP

// includes  -------------------------------------------------------------------
#include <gua/utils/Singleton.hpp>
#include <gua/platform.hpp>

#include <string>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// -----------------------------------------------------------------------------
class GUA_DLL Paths : public Singleton<Paths>
{
    ///////////////////////////////////////////////////////////////////////////////
    // ----------------------------------------------------------- public interface
  public:
    // ------------------------------------------------------------ public methods
    void init(int argc, char** argv);
    void clean_up();

    std::string tmp_file(std::string const& suffix = "tmp") const;
    std::string resource(std::string const& type, std::string const& file) const;
    std::string make_absolute(std::string const& file) const;
    std::string get_extension(std::string const& file) const;

    friend class Singleton<Paths>;

    ///////////////////////////////////////////////////////////////////////////////
    // ---------------------------------------------------------- private interface
  private:
    // this class is a Singleton --- private c'tor and d'tor
    Paths();
    ~Paths() {}

    std::string executable_;
};

} // namespace gua

#endif // GUA_GUI_PATHS_HPP
