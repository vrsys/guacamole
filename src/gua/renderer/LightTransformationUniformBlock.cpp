#include <gua/renderer/LightTransformationUniformBlock.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <gua/databases/TextureDatabase.hpp>

namespace gua
{
LightTransformationUniformBlock::LightTransformationUniformBlock(scm::gl::render_device_ptr const& device) { uniform_block_ = scm::gl::make_uniform_block<LightTransformationBlock>(device); }

LightTransformationUniformBlock::~LightTransformationUniformBlock() { uniform_block_.reset(); }

void LightTransformationUniformBlock::update(RenderContext const& context, std::vector<math::mat4f> const& light_mvp_matrices)
{

    uniform_block_.begin_manipulation(context.render_context);
    {
        memcpy(&(uniform_block_->light_mvp_matrices[0]), light_mvp_matrices.data(), light_mvp_matrices.size() * sizeof(math::mat4f) );
    }
    uniform_block_.end_manipulation();
}


#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
void LightTransformationUniformBlock::update(RenderContext const& context, std::vector<math::mat4f> const& light_mvp_matrices, std::vector<math::mat4f> const& secondary_light_mvp_matrices)
{
    uniform_block_.begin_manipulation(context.render_context);
    {
        memcpy(&(uniform_block_->light_mvp_matrices[0]), light_mvp_matrices.data(), light_mvp_matrices.size() * sizeof(math::mat4f) );
        memcpy(&(uniform_block_->secondary_light_mvp_matrices[0]), secondary_light_mvp_matrices.data(), secondary_light_mvp_matrices.size() * sizeof(math::mat4f) );
    }
    uniform_block_.end_manipulation();
}
#endif

} // namespace gua


