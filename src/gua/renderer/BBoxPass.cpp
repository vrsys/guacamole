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
#include <gua/config.hpp>
#include <gua/renderer/BBoxPass.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

namespace gua
{
BBoxPassDescription::BBoxPassDescription() : PipelinePassDescription()
{
    vertex_shader_ = "shaders/bbox.vert";
    geometry_shader_ = "shaders/bbox.geom";
    fragment_shader_ = "shaders/bbox.frag";
    private_.name_ = "BBoxPass";

    private_.writes_only_color_buffer_ = false;
    private_.rendermode_ = RenderMode::Callback;

    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(true, true, scm::gl::COMPARISON_LESS, true, 1, 0, scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)));

    private_.rasterizer_state_desc_ =
        boost::make_optional(scm::gl::rasterizer_state_desc(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0f, false, true, scm::gl::point_raster_state(true)));
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> BBoxPassDescription::make_copy() const { return std::make_shared<BBoxPassDescription>(*this); }

PipelinePass BBoxPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    auto count = 1;
    scm::gl::buffer_ptr buffer_ = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_DYNAMIC_DRAW, count * 2 * sizeof(math::vec3f), 0);
    scm::gl::vertex_array_ptr vao_ =
        ctx.render_device->create_vertex_array(scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, 2 * sizeof(math::vec3f))(0, 1, scm::gl::TYPE_VEC3F, 2 * sizeof(math::vec3f)), {buffer_});

    private_.process_ = [buffer_, vao_](PipelinePass&, PipelinePassDescription const&, Pipeline& pipe) {
        auto const& scene = *(pipe.current_viewstate().scene);
        auto count(scene.bounding_boxes.size());

        if(count < 1)
            return;
        // else
        RenderContext const& ctx(pipe.get_context());

        ctx.render_device->resize_buffer(buffer_, count * 2 * sizeof(math::vec3f));

        {
            auto data = static_cast<math::vec3f*>(ctx.render_context->map_buffer(buffer_, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER));

            for(unsigned int bounding_box_idx = 0; bounding_box_idx < count; ++bounding_box_idx)
            {
                data[2 * bounding_box_idx]     = math::vec3f(scene.bounding_boxes[bounding_box_idx].min);
                data[2 * bounding_box_idx + 1] = math::vec3f(scene.bounding_boxes[bounding_box_idx].max);
            }

            ctx.render_context->unmap_buffer(buffer_);
        }

        auto& target = *pipe.current_viewstate().target;
        bool write_depth = true;
        target.bind(ctx, write_depth);


        bool is_instanced_side_by_side_enabled = false;
        bool is_hardware_multi_view_rendering_enabled = false;

#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
        auto const& camera = (pipe.current_viewstate().camera);

        if( gua::CameraMode::BOTH == camera.config.get_mono_mode() ) {
            auto associated_window = gua::WindowDatabase::instance()->lookup(camera.config.output_window_name());//->add left_output_window
            
            if(associated_window->config.get_stereo_mode() == StereoMode::SIDE_BY_SIDE_SOFTWARE_MULTI_VIEW_RENDERING) {
                is_instanced_side_by_side_enabled = true;
            }
            if(associated_window->config.get_stereo_mode() == StereoMode::SIDE_BY_SIDE_HARDWARE_MULTI_VIEW_RENDERING) {
                is_hardware_multi_view_rendering_enabled = true;
            }
        }
#endif

    if(is_instanced_side_by_side_enabled || is_hardware_multi_view_rendering_enabled) {
        target.set_side_by_side_viewport_array(ctx);
    } else {   //std::cout << "Setting side by side viewport for bounding box renderer" << std::endl;
        target.set_viewport(ctx);
    }

        ctx.render_context->bind_vertex_array(vao_);
        ctx.render_context->apply();

        assert(count < std::numeric_limits<unsigned>::max());



        // for now only perform software mvr here
        if(is_instanced_side_by_side_enabled || is_hardware_multi_view_rendering_enabled) {
            ctx.render_context->draw_arrays_instanced(scm::gl::PRIMITIVE_POINT_LIST, 0, unsigned(count), 2);
        } else { // investigate later what happens with gl_view_id_ovr here
            ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, unsigned(count));
        }

        target.unbind(ctx);
        ctx.render_context->reset_state_objects();
        ctx.render_context->sync();

    };

    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}

} // namespace gua
