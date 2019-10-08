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
#include <gua/databases/GeometryDescription.hpp>

#include <string>
#include <gua/utils/Logger.hpp>

// external headers
#include <boost/lexical_cast.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////
GeometryDescription::GeometryDescription(std::string const& type, std::string const& filename, unsigned id, unsigned flags) : type_(type), filename_(filename), id_(id), flags_(flags)
{
    unique_key_ = type + "|" + filename + "|" + std::to_string(id) + "|" + std::to_string(flags);
}

////////////////////////////////////////////////////////////////////////////////
GeometryDescription::GeometryDescription(std::string const& unique_key) : unique_key_(unique_key)
{
    std::istringstream sstr(unique_key_);
    std::string token;

    try
    {
        std::getline(sstr, type_, '|');
        std::getline(sstr, filename_, '|');

        std::getline(sstr, token, '|');
        id_ = boost::lexical_cast<unsigned>(token);

        std::getline(sstr, token, '|');
        flags_ = boost::lexical_cast<unsigned>(token);
    }
    catch(std::exception& e)
    {
        Logger::LOG_WARNING << "GeometryDescription(std::string const& unique_key): Failed to tokenize unique key." << e.what() << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////////////
std::string const& GeometryDescription::type() const { return type_; }

////////////////////////////////////////////////////////////////////////////////
std::string const& GeometryDescription::filepath() const { return filename_; }

////////////////////////////////////////////////////////////////////////////////
unsigned GeometryDescription::flags() const { return flags_; }

////////////////////////////////////////////////////////////////////////////////
unsigned GeometryDescription::id() const { return flags_; }

////////////////////////////////////////////////////////////////////////////////
std::string const& GeometryDescription::unique_key() const { return unique_key_; }

} // namespace gua
