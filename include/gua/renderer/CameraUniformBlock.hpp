#ifndef GUA_CAMERA_UNIFORM_BLOCK_HPP
#define GUA_CAMERA_UNIFORM_BLOCK_HPP

#include <gua/math/math.hpp>
#include <gua/renderer/Frustum.hpp>
#include <gua/renderer/RenderContext.hpp>

#include <scm/gl_core/render_device/render_device_fwd.h>
#include <scm/gl_core/buffer_objects/uniform_buffer_adaptor.h>

namespace gua
{
class CameraUniformBlock
{
  public:
    struct CameraBlock
    {
        math::mat4f view;
        math::mat4f projection;
        math::mat4f projection_inverse;
        math::mat4f projection_view_inverse;
        math::vec4f position;
        math::vec4f clipping_planes[64];
        math::vec2i resolution;
        math::vec2ui noise_texture;
        math::vec4f cyclops_position;
        int clipping_plane_count;
        int view_id;
        float clip_near;
        float clip_far;
    };

    using block_type = scm::gl::uniform_block<CameraBlock>;

    CameraUniformBlock(scm::gl::render_device_ptr const& device);
    ~CameraUniformBlock();

    void
    update(RenderContext const& context, Frustum const& cam, math::vec3 const& cyclops_position, std::vector<math::vec4> const& clipping_planes, int view_id, math::vec2ui const& screen_resolution);

	void updateHMD(RenderContext const& context, Frustum const& cam, math::mat4 const& camera_parents_transform,
                   math::vec3 const& cyclops_position, std::vector<math::vec4> const& clipping_planes, int view_id,
                   math::vec2ui const& screen_resolution);

    inline const block_type& block() const { return uniform_block_; }

  private:
    block_type uniform_block_;
    math::vec2ui noise_texture_;
};

} // namespace gua

#endif // #ifndef GUA_CAMERA_UNIFORM_BLOCK_HPP
