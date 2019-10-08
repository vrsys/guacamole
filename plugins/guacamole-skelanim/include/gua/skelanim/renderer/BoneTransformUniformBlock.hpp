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

#ifndef GUA_BONE_TRANSFORM_UNIFORM_BLOCK_HPP
#define GUA_BONE_TRANSFORM_UNIFORM_BLOCK_HPP

// guacamole headers
#include <gua/skelanim/platform.hpp>
#include <gua/math/math.hpp>

// external headers
#include <scm/gl_core/buffer_objects/uniform_buffer_adaptor.h>

namespace gua
{
const std::size_t NUM_MAX_BONES = 100;
/**
 * @brief holds the transformations of all bones
 * of skeletalanimationnodes
 */
class GUA_SKELANIM_DLL BoneTransformUniformBlock
{
  public:
    struct BoneTransformBlock
    {
        math::mat4f transforms[NUM_MAX_BONES];
    };

    using block_type = scm::gl::uniform_block<BoneTransformBlock>;

    BoneTransformUniformBlock(scm::gl::render_device_ptr const& device);
    ~BoneTransformUniformBlock();

    void update(scm::gl::render_context_ptr const& context, std::vector<math::mat4f> const& new_transforms);

    inline const block_type& block() const { return uniform_block_; }

  private:
    block_type uniform_block_;
};

} // namespace gua

#endif // #ifndef GUA_BONE_TRANSFORM_UNIFORM_BLOCK_HPP
