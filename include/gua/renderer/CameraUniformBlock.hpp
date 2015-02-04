#ifndef GUA_CAMERA_UNIFORM_BLOCK_HPP
#define GUA_CAMERA_UNIFORM_BLOCK_HPP

#include <gua/math/math.hpp>
#include <gua/renderer/Frustum.hpp>

#include <scm/gl_core/render_device/render_device_fwd.h>
#include <scm/gl_core/buffer_objects/uniform_buffer_adaptor.h>

namespace gua {

class CameraUniformBlock
{
public:
  struct CameraBlock {
    math::mat4  view;
    math::mat4  projection;
    math::mat4  projection_inverse;
    math::mat4  projection_view_inverse;
    math::vec4  position;
    int         view_id;
    float       clip_near;
    float       clip_far;
  };

  typedef scm::gl::uniform_block<CameraBlock> block_type;

  CameraUniformBlock(scm::gl::render_device_ptr const& device);
  ~CameraUniformBlock();

  void update(scm::gl::render_context_ptr const& context, Frustum const& cam, int view_id);

  inline const block_type&   block() const { return uniform_block_; }

private:
  block_type          uniform_block_;
};

} // namespace gua {

#endif // #ifndef GUA_CAMERA_UNIFORM_BLOCK_HPP
