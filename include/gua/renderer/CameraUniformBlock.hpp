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
    math::mat4f  view;
    math::mat4f  projection;
    math::mat4f  projection_inverse;
    math::mat4f  projection_view_inverse;
    math::vec4f  position;
    math::vec4f  clipping_planes[64];
    math::vec2i  resolution;
    int          clipping_plane_count;
    int          view_id;
    float        clip_near;
    float        clip_far;
    bool         rendering_shadows;
  };

  typedef scm::gl::uniform_block<CameraBlock> block_type;

  CameraUniformBlock(scm::gl::render_device_ptr const& device);
  ~CameraUniformBlock();

  void update(scm::gl::render_context_ptr const& context, Frustum const& cam,
              std::vector<math::vec4f> const& clipping_planes,
              int view_id, math::vec2ui const& screen_resolution,
              bool rendering_shadows);

  inline const block_type&   block() const { return uniform_block_; }

private:
  block_type          uniform_block_;
};

} // namespace gua {

#endif // #ifndef GUA_CAMERA_UNIFORM_BLOCK_HPP
