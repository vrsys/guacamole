#ifndef GUA_LIGHT_TRANSFORMATION_UNIFORM_BLOCK_HPP
#define GUA_LIGHT_TRANSFORMATION_UNIFORM_BLOCK_HPP

#include <gua/config.hpp>
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
#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
        math::mat4f secondary_light_mvp_matrices[32];
#endif
    };

    using block_type = scm::gl::uniform_block<LightTransformationBlock>;

    LightTransformationUniformBlock(scm::gl::render_device_ptr const& device);
    ~LightTransformationUniformBlock();

    void
    update(RenderContext const& context, std::vector<math::mat4f> const& light_mvp_matrices);

#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
    void
    update(RenderContext const& context, std::vector<math::mat4f> const& light_mvp_matrices, std::vector<math::mat4f> const& secondary_light_mvp_matrices);
#endif
    inline const block_type& block() const { return uniform_block_; }

  private:
    block_type uniform_block_;
};

} // namespace gua

#endif // #ifndef GUA_LIGHT_TRANSFORMATION_UNIFORM_BLOCK_HPP
