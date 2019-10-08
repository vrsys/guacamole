#ifndef GUA_PHYSICAL_TEXTURE_UNIFORM_BLOCK_HPP
#define GUA_PHYSICAL_TEXTURE_UNIFORM_BLOCK_HPP

#include <gua/math/math.hpp>
#include <gua/renderer/RenderContext.hpp>

#include <scm/gl_core/render_device/render_device_fwd.h>
#include <scm/gl_core/buffer_objects/uniform_buffer_adaptor.h>

namespace gua
{
class PhysicalTextureUniformBlock
{
  public:
    struct PhysicalTextureBlock
    {
        uint64_t texture_handle;
        math::vec2ui tile_size;
        math::vec2ui tile_padding;
        math::vec2ui texture_dimensions;
    };

    using block_type = scm::gl::uniform_block<PhysicalTextureBlock>;

    PhysicalTextureUniformBlock(scm::gl::render_device_ptr const& device);
    ~PhysicalTextureUniformBlock();

    void update(RenderContext const& context, uint64_t tex_handle, math::vec2ui const& t_size, math::vec2ui const& t_padding, math::vec2ui const& dims);

    inline const block_type& block() const { return uniform_block_; }

  private:
    block_type uniform_block_;
};

} // namespace gua

#endif // #ifndef GUA_PHYSICAL_TEXTURE_UNIFORM_BLOCK_HPP
