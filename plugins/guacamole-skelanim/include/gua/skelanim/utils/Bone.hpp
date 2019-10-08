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

#ifndef GUA_BONE_HPP
#define GUA_BONE_HPP

// guacamole headers
#include <gua/config.hpp>
#include <gua/utils/fbxfwd.hpp>
#include <gua/skelanim/platform.hpp>

// external headers
#include <scm/gl_core.h>

struct aiNode;

namespace gua
{
/**
 * @brief represents one node in skeletal hierarchy
 * @details has methods to traverse skeleton hierarchy
 */
struct GUA_SKELANIM_DLL Bone
{
  public:
    Bone();
    Bone(std::string const& name, scm::math::mat4f const& idle, scm::math::mat4f const& offset = scm::math::mat4f::identity(), std::vector<unsigned> childs = std::vector<unsigned>{});
    Bone(aiNode const& node);

#ifdef GUACAMOLE_FBX
    Bone(FbxNode& node);
#endif

    std::string name;
    std::vector<unsigned> children;
    // idle/reference pose matrix
    scm::math::mat4f idle_matrix;
    // transforms to bone space
    scm::math::mat4f offset_matrix;
};

} // namespace gua

#endif // GUA_BONE_HPP
