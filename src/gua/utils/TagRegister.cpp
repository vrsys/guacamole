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

#include <gua/utils/TagRegister.hpp>

#include <gua/utils/Logger.hpp>

namespace gua
{
namespace utils
{
////////////////////////////////////////////////////////////////////////////////

std::bitset<GUA_MAX_TAG_COUNT> const& TagRegister::get_tag(std::string const& tag)
{
    if(registered_tags_.find(tag) == registered_tags_.end())
    {
        auto tag_count(registered_tags_.size());

        if(tag_count < GUA_MAX_TAG_COUNT)
        {
            std::bitset<GUA_MAX_TAG_COUNT> new_set;
            new_set.set(tag_count);

            registered_tags_[tag] = new_set;
        }
        else
        {
            Logger::LOG_WARNING << "Unable to add new tag: Maximum number of tags is " << GUA_MAX_TAG_COUNT << "!" << std::endl;
            return default_tag_;
        }
    }

    return registered_tags_[tag];
}

////////////////////////////////////////////////////////////////////////////////

std::vector<std::string> const TagRegister::get_tag_strings(std::bitset<GUA_MAX_TAG_COUNT> const& tags)
{
    std::vector<std::string> tag_strings;

    for(auto const& tag : registered_tags_)
    {
        if((tag.second & tags).any())
        {
            tag_strings.push_back(tag.first);
        }
    }

    return tag_strings;
}

} // namespace utils
} // namespace gua
