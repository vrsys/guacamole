#include <gua/renderer/LightTable.hpp>

namespace gua {

void LightTable::remove_buffers(RenderContext const& ctx) {
  if (light_bitset_) {
    light_bitset_->make_non_resident(ctx);
  }
}

void LightTable::invalidate(RenderContext const& ctx,
                            math::vec2ui const& resolution,
                            LightTable::array_type const& lights) {

  lights_num_ = lights.size();

  if (!lights_num_) {
    return;
  }

  const int max_tex3d_size = ctx.render_device->capabilities()._max_texture_3d_size;
  const int tile_power = 2; // TODO: should be passed to shader

  float width  = std::ceil(resolution.x / std::pow(2, tile_power));
  float height = std::ceil(resolution.y / std::pow(2, tile_power));
  unsigned light_bitset_words = ((lights_num_ - 1) / 32) + 1;

  if (   width > max_tex3d_size
      || height > max_tex3d_size
      || light_bitset_words > max_tex3d_size) {
    Logger::LOG_ERROR << "Dimensions of light table cannot be greater than "
                      << max_tex3d_size << " in size" << std::endl;
  }

  // create bitset if necessary
  if (  !light_bitset_ 
      || light_bitset_->width() < width
      || light_bitset_->height() < height
      || light_bitset_words > light_bitset_words_) {
    scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_NEAREST,
                                      scm::gl::WRAP_MIRRORED_REPEAT,
                                      scm::gl::WRAP_MIRRORED_REPEAT);

    if (light_bitset_) {
      light_bitset_->make_non_resident(ctx);
      light_bitset_.reset();
    }
    light_bitset_ = std::make_shared<Texture3D>(width, height, light_bitset_words,
                                                scm::gl::FORMAT_R_32UI, 1, state);
    light_bitset_words_ = light_bitset_words;
    Logger::LOG_DEBUG << "Light bitset allocation for " << light_bitset_words << " words" << std::endl;
    Logger::LOG_DEBUG << "Size of LightBlock: " << sizeof(LightBlock) << std::endl;
  }


  // clear bitset
  ctx.render_context->clear_image_data(light_bitset_->get_buffer(ctx), 0, 
                                       scm::gl::FORMAT_R_32UI, 0);

  // upload light UBO
  bool needs_update(false);
  if (uniform_block_.array_size() < lights_num_) {
    uniform_block_ = scm::gl::make_uniform_block_array<LightBlock>(ctx.render_device, 
                                                                   lights_num_);
    Logger::LOG_DEBUG << "Create light UBO for " << lights_num_ << " elements" << std::endl;
    needs_update = true;
  }

  for (size_t i = 0; i < lights_num_; ++i) {
    if (uniform_block_[i] != lights[i]) {
      uniform_block_[i] = lights[i];
      needs_update = true;
    }
  }

  if (needs_update) {
    uniform_block_.begin_manipulation(ctx.render_context);
    uniform_block_.end_manipulation();
    Logger::LOG_DEBUG << "Light data upload" << std::endl;
  }
}

} // namespace gua
