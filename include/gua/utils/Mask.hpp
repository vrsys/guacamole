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

#ifndef GUA_RENDER_MASK_HPP
#define GUA_RENDER_MASK_HPP

#include <gua/platform.hpp>
#include <gua/utils/TagList.hpp>

// external headers
#include <string>
#include <vector>
#include <set>

namespace gua
{
/**
 * RenderMasks are used to display only parts of a scene graph.
 *
 * A Mask is a string like "floor & lights". This Mask will only
 * display nodes which are in both groups; "floor" and "lights" --- all other
 * nodes are discarded. If the mask is "floor | lights" all nodes which are in
 * one (or both) groups are shown. Supported operators are:
 * & --- and
 * | --- or
 * ! --- not
 * ( --- opening parenthesis
 * ) --- closing parenthesis
 * Whitespaces are ignored and all other characters are treated as group names.
 */
class GUA_DLL Mask
{
  public:
    /**
     * Constructor.
     *
     * This constructs an Mask.
     *
     * \param render_mask      The Mask's string representation.
     */
    Mask(std::vector<std::string> const& whitelist_tags = std::vector<std::string>(), std::vector<std::string> const& blacklist_tags = std::vector<std::string>());

    /**
     * Checks a given list of groups against this mask.
     *
     * \param groups           A set of groups.
     * \return                 true, if the given list of groups is
     *                         supported by this Mask.
     */
    bool check(gua::utils::TagList const& tags) const;

    gua::utils::TagList whitelist;
    gua::utils::TagList blacklist;

    void set_user_data(void* data) { user_data_ = data; }

    void* get_user_data() const { return user_data_; }

    bool operator==(Mask const& other) const { return whitelist == other.whitelist && blacklist == other.blacklist; }

  private:
    void* user_data_ = nullptr;
};

} // namespace gua

#endif // GUA_RENDER_MASK_HPP
