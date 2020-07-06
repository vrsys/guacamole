#include <gua/renderer/LightTable.hpp>

namespace gua
{
void LightTable::remove_buffers(RenderContext const& ctx)
{
    if(light_bitset_)
    {
        light_bitset_->make_non_resident(ctx);
    }

#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
    if(secondary_light_bitset_)
    {
        secondary_light_bitset_->make_non_resident(ctx);
    }
#endif
}

math::vec2ui LightTable::invalidate(RenderContext const& ctx, math::vec2ui const& resolution, LightTable::array_type const& lights, int tile_power, int sun_lights_num)
{
    sun_lights_num_ = sun_lights_num;
    lights_num_ = lights.size();

    unsigned width{};
    unsigned height{};

    if(tile_power > 0)
    {
        width = std::ceil(float(resolution.x) / std::pow(2, tile_power));
        height = std::ceil(float(resolution.y) / std::pow(2, tile_power));
    }
    else
    {
        width = resolution.x;
        height = resolution.y;
    }

    if(!lights_num_)
    {
        return math::vec2ui(width, height);
    }

    unsigned light_bitset_words = ((lights_num_ - 1) / 32) + 1;

    const unsigned int max_tex3d_size = ctx.render_device->capabilities()._max_texture_3d_size;

    if(width > max_tex3d_size || height > max_tex3d_size || light_bitset_words > max_tex3d_size)
    {
        Logger::LOG_ERROR << "Dimensions of light table cannot be greater than " << max_tex3d_size << " in size" << std::endl;
    }

    // create bitset if necessary
    if(!light_bitset_ || light_bitset_->width() < width || light_bitset_->height() < height || light_bitset_words > light_bitset_words_)
    {
        scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_MIRRORED_REPEAT, scm::gl::WRAP_MIRRORED_REPEAT);

        if(light_bitset_)
        {
            light_bitset_->make_non_resident(ctx);
            light_bitset_.reset();
        }


#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
        if(secondary_light_bitset_) {
            secondary_light_bitset_->make_non_resident(ctx);
            secondary_light_bitset_.reset();
        }
#endif
        light_bitset_ = std::make_shared<Texture3D>(width, height, light_bitset_words, scm::gl::FORMAT_R_32UI, 1, state);

#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
        secondary_light_bitset_ = std::make_shared<Texture3D>(width, height, light_bitset_words, scm::gl::FORMAT_R_32UI, 1, state);
#endif  

        light_bitset_words_ = light_bitset_words;
        Logger::LOG_DEBUG << "Light bitset allocation for " << light_bitset_words << " words" << std::endl;
        Logger::LOG_DEBUG << "Size of LightBlock: " << sizeof(LightBlock) << std::endl;
    }

    // clear bitset
    ctx.render_context->clear_image_sub_data(
        light_bitset_->get_buffer(ctx), scm::gl::texture_region(math::vec3ui(0, 0, 0), math::vec3ui(width, height, light_bitset_words_)), 0, scm::gl::FORMAT_R_32UI, 0);

#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
    ctx.render_context->clear_image_sub_data(
        secondary_light_bitset_->get_buffer(ctx), 
        scm::gl::texture_region(math::vec3ui(0, 0, 0), math::vec3ui(width, height, light_bitset_words_)), 
        0, scm::gl::FORMAT_R_32UI, 0);
#endif


    // upload light UBO
    bool needs_update(false);
    if(uniform_block_.array_size() < lights_num_)
    {
        uniform_block_ = scm::gl::make_uniform_block_array<LightBlock>(ctx.render_device, lights_num_);
        Logger::LOG_DEBUG << "Create light UBO for " << lights_num_ << " elements" << std::endl;
        needs_update = true;
    }

    for(size_t i = 0; i < lights_num_; ++i)
    {
        if(uniform_block_[i] != lights[i])
        {
            uniform_block_[i] = lights[i];
            needs_update = true;
        }
    }

    if(needs_update)
    {
        uniform_block_.begin_manipulation(ctx.render_context);
        uniform_block_.end_manipulation();
        // Logger::LOG_DEBUG << "Light data upload" << std::endl;
    }
    return math::vec2ui(width, height);
}

} // namespace gua
