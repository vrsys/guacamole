#ifndef GUA_LIGHT_VISIBILITY_RENDERER_HPP
#define GUA_LIGHT_VISIBILITY_RENDERER_HPP

#include <gua/config.hpp>

#include <gua/renderer/PipelinePass.hpp>
#include <gua/renderer/LightTable.hpp>

namespace gua
{
class Pipeline;

class LightVisibilityRenderer
{
  public:
    LightVisibilityRenderer() {}

    void render(PipelinePass& pass, Pipeline& pipe, int tile_power, unsigned ms_sample_count, bool enable_conservative, bool enable_fullscreen_fallback);

  private:
    void draw_lights(Pipeline& pipe, std::vector<math::mat4>& transforms, LightTable::array_type& lights) const;

    void prepare_light_table(Pipeline& pipe, std::vector<math::mat4>& transforms, LightTable::array_type& lights, unsigned& sun_lights_num) const;

    void add_pointlight(Pipeline& pipe, node::LightNode& light, LightTable::LightBlock& light_block, LightTable::array_type& lights, std::vector<math::mat4>& light_transforms) const;

    void add_spotlight(Pipeline& pipe, node::LightNode& light, LightTable::LightBlock& light_block, LightTable::array_type& lights, std::vector<math::mat4>& light_transforms) const;

    void add_sunlight(Pipeline& pipe, node::LightNode& light, LightTable::LightBlock& light_block, LightTable::array_type& lights, std::vector<math::mat4>& light_transforms) const;

    scm::gl::frame_buffer_ptr empty_fbo_ = nullptr;
    scm::gl::texture_2d_ptr empty_fbo_color_attachment_ = nullptr;
};

} // namespace gua

#endif // GUA_LIGHT_VISIBILITY_RENDERER_HPP
