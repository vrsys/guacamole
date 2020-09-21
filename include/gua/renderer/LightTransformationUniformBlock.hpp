#ifndef GUA_LIGHT_TRANSFORMATION_UNIFORM_BLOCK_HPP
#define GUA_LIGHT_TRANSFORMATION_UNIFORM_BLOCK_HPP

#include <gua/math/math.hpp>
#include <gua/renderer/RenderContext.hpp>

#include <scm/gl_core/render_device/render_device_fwd.h>
#include <scm/gl_core/buffer_objects/uniform_buffer_adaptor.h>

namespace gua
{
class LightTransformationUniformBlock
{
  public:
    struct LightTransformationBlock
    {
        math::mat4f light_mvp_matrices[32];
    };

    using block_type = scm::gl::uniform_block<LightTransformationBlock>;

    LightTransformationUniformBlock(scm::gl::render_device_ptr const& device);
    ~LightTransformationUniformBlock();

    void
    update(RenderContext const& context, std::vector<math::mat4f> const& light_mvp_matrices);//Frustum const& cam, math::vec3 const& cyclops_position, std::vector<math::vec4> const& clipping_planes, int view_id, math::vec2ui const& screen_resolution);

    inline const block_type& block() const { return uniform_block_; }

  private:
    block_type uniform_block_;
};

} // namespace gua

#endif // #ifndef GUA_LIGHT_TRANSFORMATION_UNIFORM_BLOCK_HPP
