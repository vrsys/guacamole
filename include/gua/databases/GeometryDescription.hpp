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
#ifndef GUA_GEOMETRY_DESCRIPTION_HPP
#define GUA_GEOMETRY_DESCRIPTION_HPP

#include <string>

#include <gua/platform.hpp>

namespace gua
{
/*
 * helper class to generate unique geometry string from parametrization and vice versa
 */
class GUA_DLL GeometryDescription
{
  public: // construction
    GeometryDescription(std::string const& type, std::string const& filename, unsigned id, unsigned flags);

    GeometryDescription(std::string const& unique_key);

  public: // methods
    std::string const& type() const;
    std::string const& filepath() const;

    unsigned flags() const;
    unsigned id() const;

    std::string const& unique_key() const;

  private:
    std::string type_;
    std::string filename_;

    unsigned id_;
    unsigned flags_;

    std::string unique_key_;
};

} // namespace gua

#endif // GUA_GEOMETRY_DESCRIPTION_HPP
