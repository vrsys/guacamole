#include <gua/renderer/PhysicalTextureUniformBlock.hpp>

namespace gua
{
PhysicalTextureUniformBlock::PhysicalTextureUniformBlock(scm::gl::render_device_ptr const& device) { uniform_block_ = scm::gl::make_uniform_block<PhysicalTextureBlock>(device); }

PhysicalTextureUniformBlock::~PhysicalTextureUniformBlock() { uniform_block_.reset(); }

void PhysicalTextureUniformBlock::update(RenderContext const& context, uint64_t tex_handle, math::vec2ui const& t_size, math::vec2ui const& t_padding, math::vec2ui const& dims)
{
    uniform_block_.begin_manipulation(context.render_context);
    {
        uniform_block_->texture_handle = tex_handle;
        uniform_block_->tile_size = t_size;
        uniform_block_->tile_padding = t_padding;
        uniform_block_->texture_dimensions = dims;
    }
    uniform_block_.end_manipulation();
}

} // namespace gua
