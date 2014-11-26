#ifndef GUA_BONE_TRANSFORM_UNIFORM_BLOCK_HPP
#define GUA_BONE_TRANSFORM_UNIFORM_BLOCK_HPP

#include <gua/math/math.hpp>
#include <gua/renderer/Frustum.hpp>

#include <scm/gl_core/render_device/render_device_fwd.h>
#include <scm/gl_core/buffer_objects/uniform_buffer_adaptor.h>

namespace gua {

class BoneTransformUniformBlock
{
public:
  struct BoneTransformBlock {
    math::mat4 transforms[100]; 
  };

  typedef scm::gl::uniform_block<BoneTransformBlock> block_type;

  BoneTransformUniformBlock(scm::gl::render_device_ptr const& device);
  ~BoneTransformUniformBlock();

  void update(scm::gl::render_context_ptr const& context, std::vector<math::mat4> new_transforms);

  inline const block_type&   block() const { return uniform_block_; }

private:
  block_type          uniform_block_;
};

} // namespace gua {

#endif // #ifndef GUA_BONE_TRANSFORM_UNIFORM_BLOCK_HPP
