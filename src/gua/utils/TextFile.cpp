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
#include <gua/utils/TextFile.hpp>
#include <gua/utils/PathParser.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>

// external headers
#include <sstream>
#include <fstream>

#include <boost/filesystem.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

TextFile::TextFile() : file_name_(""), content_(""), is_loaded_(false) {}

////////////////////////////////////////////////////////////////////////////////

TextFile::TextFile(std::string const& file_name) : file_name_(file_name), content_(""), is_loaded_(false) {}

////////////////////////////////////////////////////////////////////////////////

bool TextFile::is_valid() const
{
    std::ifstream file(file_name_.c_str());

    if(file.fail())
        return false;

    file.close();
    return true;
}

////////////////////////////////////////////////////////////////////////////////

std::string const& TextFile::get_content() const
{
    if(is_loaded_)
        return content_;

    is_loaded_ = true;

    std::ifstream ifs(file_name_.c_str());
    if(!ifs)
    {
        Logger::LOG_WARNING << "Cannot open file \"" << file_name_ << "\"!" << std::endl;
        return content_;
    }

    std::stringstream oss;
    oss << ifs.rdbuf();

    if(!ifs && !ifs.eof())
    {
        Logger::LOG_WARNING << "Error reading file \"" << file_name_ << "\"!" << std::endl;
        return content_;
    }

    content_ = std::string(oss.str());
    return content_;
}

////////////////////////////////////////////////////////////////////////////////

void TextFile::set_content(std::string const& content)
{
    content_ = content;
    is_loaded_ = true;
}

////////////////////////////////////////////////////////////////////////////////

bool TextFile::save(bool create_subdirs) const
{
    if(!is_loaded_)
    {
        Logger::LOG_WARNING << "Unable to save file \"" << file_name_ << "\"! No content has been set." << std::endl;
        return false;
    }

    if(create_subdirs)
    {
        PathParser parser;
        parser.parse(file_name_);
        auto path(parser.get_parsed_path());

        std::string current_path("");
        for(unsigned i(0); i < path.size() - 1; ++i)
        {
            current_path += path[i] + "/";

            if(!boost::filesystem::exists(current_path.c_str()) || boost::filesystem::is_directory(current_path.c_str()))
            {
                boost::filesystem::create_directory(current_path.c_str());
            }
            /*if (!opendir(current_path.c_str()))
                system(("mkdir " + current_path).c_str());*/
        }
    }

    std::ofstream ofs(file_name_);
    if(!ofs)
    {
        Logger::LOG_WARNING << "Cannot open file \"" << file_name_ << "\"!" << std::endl;
        return false;
    }

    ofs << content_;
    ofs.close();

    return true;
}

////////////////////////////////////////////////////////////////////////////////

void TextFile::remove()
{
    std::remove(file_name_.c_str());
    content_ = "";
    is_loaded_ = false;
}

////////////////////////////////////////////////////////////////////////////////

std::string const& TextFile::get_file_name() const { return file_name_; }

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
