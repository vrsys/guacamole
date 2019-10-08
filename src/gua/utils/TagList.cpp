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

#include <gua/utils/TagList.hpp>

#include <gua/utils/Logger.hpp>

namespace gua
{
namespace utils
{
/////////////////////////////////////////////////////////////////////////////////
TagList::TagList(std::vector<std::string> const& tags) { add_tags(tags); }

/////////////////////////////////////////////////////////////////////////////////

void TagList::add_tag(std::string const& tag)
{
    auto new_tag(gua::utils::TagRegister::instance()->get_tag(tag));
    if(new_tag.any())
    {
        tags_ |= new_tag;
    }
}

////////////////////////////////////////////////////////////////////////////////

void TagList::add_tags(std::vector<std::string> const& tags)
{
    for(auto const& tag : tags)
    {
        add_tag(tag);
    }
}

////////////////////////////////////////////////////////////////////////////////

void TagList::remove_tag(std::string const& tag)
{
    if(tags_.any())
    {
        auto tag_to_remove(gua::utils::TagRegister::instance()->get_tag(tag));
        if(tag_to_remove.any())
        {
            tags_ &= tag_to_remove.flip();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void TagList::remove_tags(std::vector<std::string> const& tags)
{
    if(tags_.any())
    {
        for(auto tag : tags)
        {
            remove_tag(tag);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void TagList::clear_tags() { tags_.reset(); }

////////////////////////////////////////////////////////////////////////////////

std::vector<std::string> const TagList::get_strings() const
{
    if(tags_.any())
    {
        return gua::utils::TagRegister::instance()->get_tag_strings(tags_);
    }

    return std::vector<std::string>();
}

////////////////////////////////////////////////////////////////////////////////

std::bitset<GUA_MAX_TAG_COUNT> const& TagList::get_bits() const { return tags_; }

} // namespace utils
} // namespace gua
