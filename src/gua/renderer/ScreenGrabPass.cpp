/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

// class header
#include <gua/renderer/ScreenGrabPass.hpp>

#include <gua/renderer/Pipeline.hpp>

namespace gua
{
ScreenGrabPassDescription::ScreenGrabPassDescription() : PipelinePassDescription(), output_prefix_(""), grab_next_(false)
{
    vertex_shader_ = "";
    geometry_shader_ = "";
    fragment_shader_ = "";
    private_.name_ = "ScreenGrabPass";

    private_.needs_color_buffer_as_input_ = true;
    private_.writes_only_color_buffer_ = true;
    private_.enable_for_shadows_ = false;
    private_.rendermode_ = RenderMode::Custom;

    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(true, true, scm::gl::COMPARISON_LESS, true, 1, 0, scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)));

    private_.rasterizer_state_desc_ =
        boost::make_optional(scm::gl::rasterizer_state_desc(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0f, false, true, scm::gl::point_raster_state(true)));
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> ScreenGrabPassDescription::make_copy() const { return std::make_shared<ScreenGrabPassDescription>(*this); }

PipelinePass ScreenGrabPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    private_.process_ = [&](PipelinePass&, PipelinePassDescription const&, Pipeline& pipe) {
        RenderContext const& ctx(pipe.get_context());

        if(grab_next_)
        {
            grab_next_ = false;
            scm::gl::texture_2d_ptr color_buffer = pipe.get_gbuffer()->get_color_buffer();
            scm::math::vec2ui dims = color_buffer->dimensions();

            auto format = color_buffer->format();

            if(format != scm::gl::data_format::FORMAT_RGB_32F)
            {
                std::cerr << "Invalid use of ScreenGrabPass with non-standard color buffer" << std::endl;
                return;
            }

            std::vector<float> host_color_buffer(dims.x * dims.y * 3);

            ctx.render_context->retrieve_texture_data(color_buffer, 0, &host_color_buffer[0]);
            ctx.render_context->sync();

            ScreenGrabJPEGSaver::get_instance()->save(output_prefix_, dims, host_color_buffer);
        }
    };

    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}
void ScreenGrabPassDescription::set_output_prefix(const std::string& output_prefix)
{
    output_prefix_ = output_prefix;
    touch();
}
void ScreenGrabPassDescription::set_grab_next(bool grab_next)
{
    grab_next_ = grab_next;
    touch();
}

} // namespace gua
