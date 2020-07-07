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
#include <gua/renderer/OcclusionSlaveResolvePass.hpp>

#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/renderer/Pipeline.hpp>

#include <gua/utils/Logger.hpp>
#include <gua/utils/NamedSharedMemoryController.hpp>

#include <boost/variant.hpp>

#include <atomic>
namespace gua
{
////////////////////////////////////////////////////////////////////////////////
OcclusionSlaveResolvePassDescription::OcclusionSlaveResolvePassDescription()
    : PipelinePassDescription(), last_rendered_view_id(std::numeric_limits<int>::max()), last_rendered_side(0), gbuffer_extraction_resolution_(scm::math::vec2ui{256, 144}),
      control_monitor_shader_stages_(), control_monitor_shader_program_(nullptr), depth_downsampling_shader_stages_(), depth_downsampling_shader_program_(nullptr),
      gpu_resources_already_created_(false)
{
    vertex_shader_ = "";
    fragment_shader_ = "";
    private_.name_ = "OcclusionSlaveResolvePass";
    private_.needs_color_buffer_as_input_ = true;
    private_.writes_only_color_buffer_ = true;
    private_.rendermode_ = RenderMode::Custom;
    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(false, false, scm::gl::COMPARISON_LESS, true, 1, 0, scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)));
}

/*void OcclusionSlaveResolvePassDescription::apply_post_render_action(RenderContext const &ctx, gua::Pipeline *pipe) const
{

      auto memory_controller = gua::NamedSharedMemoryController::instance_shared_ptr();
      //std::cout << "Writing depth buffer to: " << depth_buffer_shared_memory_name << "\n";

      memory_controller->add_read_only_memory_segment("DEPTH_FEEDBACK_SEGMENT");


      // write depth res x & y
      memory_controller->register_remotely_constructed_object_on_segment("DEPTH_FEEDBACK_SEGMENT", "DEPTH_BUFFER_RES_X");
      memory_controller->set_value_for_named_object<std::atomic_int, int>("DEPTH_BUFFER_RES_X", gbuffer_extraction_resolution_[0]);

      memory_controller->register_remotely_constructed_object_on_segment("DEPTH_FEEDBACK_SEGMENT", "DEPTH_BUFFER_RES_Y");
      memory_controller->set_value_for_named_object<std::atomic_int, int>("DEPTH_BUFFER_RES_Y", gbuffer_extraction_resolution_[1]);

      // signal reconstruction server
      memory_controller->register_remotely_constructed_object_on_segment("DEPTH_FEEDBACK_SEGMENT", "DEPTH_FEEDBACK_SEMAPHOR");
      memory_controller->set_value_for_named_object<std::atomic_int, int>("DEPTH_FEEDBACK_SEMAPHOR", 2);
      std::cout << "Would signal now!\n";

}*/

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<PipelinePassDescription> OcclusionSlaveResolvePassDescription::make_copy() const { return std::make_shared<OcclusionSlaveResolvePassDescription>(*this); }

void OcclusionSlaveResolvePassDescription::create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims)
{
    ResourceFactory factory;

    // init control monitor shaders
    {
        control_monitor_shader_stages_.clear();
        control_monitor_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/common/fullscreen_quad.vert")));
        control_monitor_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/occlusion_resolve_control_monitor.frag")));

        auto new_program = std::make_shared<ShaderProgram>();
        new_program->set_shaders(control_monitor_shader_stages_);
        control_monitor_shader_program_ = new_program;
    }

    // init depth downsampling shaders
    {
        depth_downsampling_shader_stages_.clear();
        depth_downsampling_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/common/fullscreen_quad.vert")));
        depth_downsampling_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/occlusion_resolve_depth_downsampling.frag")));

        auto new_program = std::make_shared<ShaderProgram>();
        new_program->set_shaders(depth_downsampling_shader_stages_);
        depth_downsampling_shader_program_ = new_program;
    }

    no_depth_test_depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(false, false, scm::gl::COMPARISON_ALWAYS);

    always_write_depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_ALWAYS);

    depth_buffer_downsampling_fbo_ = ctx.render_device->create_frame_buffer();
    downsampled_depth_attachment_ = ctx.render_device->create_texture_2d(gbuffer_extraction_resolution_, scm::gl::FORMAT_R_32F, 1, 1, 1);

    depth_buffer_downsampling_fbo_->attach_color_buffer(0, downsampled_depth_attachment_);

    nearest_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);

    /*
        depth_buffer_       = ctx.render_device->create_texture_2d(resolution, scm::gl::FORMAT_D24_S8,  1);
      ctx.render_context->make_resident(depth_buffer_, sampler_state_);

      fbo_read_ = ctx.render_device->create_frame_buffer();
      fbo_read_->attach_color_buffer(0, color_buffer_read_,0,0);
      fbo_read_->attach_color_buffer(1, pbr_buffer_, 0, 0);
      fbo_read_->attach_color_buffer(2, normal_buffer_,0,0);
      fbo_read_->attach_color_buffer(3, flags_buffer_,0,0);
      fbo_read_->attach_depth_stencil_buffer(depth_buffer_,0,0);
    */
}

////////////////////////////////////////////////////////////////////////////////
PipelinePass OcclusionSlaveResolvePassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
#if not defined(__WIN32__) && not defined(_WIN32) && not defined(_WIN64)
    private_.process_ = [&](PipelinePass& pass, PipelinePassDescription const& desc, Pipeline& pipe, bool render_multiview, bool use_hardware_mvr) {
        // auto& target = pipe.current_viewstate().target;
        // auto& gua_depth_buffer = target->get_depth_buffer();

        auto const& camera = pipe.current_viewstate().camera;

        scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();

        // custom render pass
        if(!gpu_resources_already_created_)
        {
            create_gpu_resources(ctx, render_target_dims);
            gpu_resources_already_created_ = true;
        }

        ////// >> perform depth downsampling

        // scm::gl::context_all_guard context_guard(ctx.render_context);
        auto& target = *pipe.current_viewstate().target;

        target.set_viewport(ctx);

        // auto& gua_depth_buffer = target.get_depth_buffer();

        auto& gua_depth_buffer = pipe.get_gbuffer()->get_depth_buffer();
        pipe.get_gbuffer()->toggle_ping_pong();
        // target.unbind(ctx);
        depth_downsampling_shader_program_->use(ctx);

        // ctx.render_context
        //  ->set_depth_stencil_state(no_depth_test_depth_stencil_state_);

        depth_buffer_downsampling_fbo_->attach_color_buffer(0, downsampled_depth_attachment_);

        ctx.render_context->set_frame_buffer(depth_buffer_downsampling_fbo_);

        ctx.render_context->bind_texture(gua_depth_buffer, nearest_sampler_state_, 0);

        depth_downsampling_shader_program_->apply_uniform(ctx, "gua_in_depth_buffer", 0);

        scm::math::vec2 downsampling_ratio = scm::math::vec2(render_target_dims) / scm::math::vec2(gbuffer_extraction_resolution_);
        depth_downsampling_shader_program_->set_uniform(ctx, downsampling_ratio, "downsampling_factors");
        depth_downsampling_shader_program_->set_uniform(ctx, render_target_dims, "original_resolution");

        ctx.render_context->apply();
        pipe.draw_quad();

        depth_downsampling_shader_program_->unuse(ctx);
        pipe.get_gbuffer()->toggle_ping_pong();

        /*
          depth_downsampling_shader_program_->use(ctx);

          for (auto const& u : desc.uniforms) {
            u.second.apply(ctx, u.first, ctx.render_context->current_program(), 0);
          }

          pipe.bind_gbuffer_input(depth_downsampling_shader_program_);

          ctx.render_context->clear_depth_stencil_buffer(depth_buffer_downsampling_fbo_);
          ctx.render_context->set_depth_stencil_state(always_write_depth_stencil_state_);
          ctx.render_context->set_frame_buffer(depth_buffer_downsampling_fbo_);



          //ctx.render_context
          //    ->bind_texture(gua_depth_buffer, nearest_sampler_state_, 0);

          //depth_downsampling_shader_program_->apply_uniform(ctx,
          //  "gua_depth_buffer", 0);


          scm::math::vec2 downsampling_ratio = scm::math::vec2(render_target_dims) / scm::math::vec2(gbuffer_extraction_resolution_);
          depth_downsampling_shader_program_->set_uniform(ctx, downsampling_ratio, "downsampling_factors");
          depth_downsampling_shader_program_->set_uniform(ctx, render_target_dims, "original_resolution");

          ctx.render_context->apply();
          pipe.draw_quad();

          ctx.render_context->reset_state_objects();

          */

        /////<<<<<<<<<<<<<<

        // scm::gl::context_all_guard context_guard(ctx.render_context);

        target.bind(ctx, !pass.writes_only_color_buffer());
        target.set_viewport(ctx);

        // scm::math::vec2 downsampling_ratio = scm::math::vec2(render_target_dims) / scm::math::vec2(gbuffer_extraction_resolution_);
        // control_monitor_shader_program_->set_uniform(ctx, downsampling_ratio, "downsampling_factors");

        control_monitor_shader_program_->use(ctx);

        for(auto const& u : desc.uniforms)
        {
            u.second.apply(ctx, u.first, ctx.render_context->current_program(), 0);
        }

        pipe.bind_gbuffer_input(control_monitor_shader_program_);
        // pipe.bind_light_table(shader_);

        // std::string gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / " + name_;
        // pipe.begin_gpu_query(ctx, gpu_query_name);

        // if (RenderMode::Callback == rendermode_) {
        //  process_(*this, desc, pipe);
        //} else { // RenderMode::Quad

        ctx.render_context->set_depth_stencil_state(no_depth_test_depth_stencil_state_);

        ctx.render_context->bind_texture(downsampled_depth_attachment_, nearest_sampler_state_, 0);
        depth_downsampling_shader_program_->apply_uniform(ctx, "downsampled_depth_buffer", 0);

        ctx.render_context->apply();
        pipe.draw_quad();
        control_monitor_shader_program_->unuse(ctx);
        //}

        // pipe.end_gpu_query(ctx, gpu_query_name);

        target.unbind(ctx);

        /////>>>>>>>>>>>>>>>>

        uint32_t pixel_size = gbuffer_extraction_resolution_[0] * gbuffer_extraction_resolution_[1];

        std::vector<float> texture_data(pixel_size, 0);

        ctx.render_context->retrieve_texture_data(downsampled_depth_attachment_, 0, (uint32_t*)&texture_data[0]);
        // pipe.get_gbuffer()->retrieve_depth_data(ctx, (uint32_t*)&texture_data[0]);

        // std::cout << "Retrieving Depth buffer data!\n";

        // uint32_t max_uint = std::numeric_limits<uint32_t>::max();

        /*for(uint32_t y_idx = 0; y_idx < gbuffer_extraction_resolution_[1]; ++y_idx ) {
          for(uint32_t x_idx = 0; x_idx < gbuffer_extraction_resolution_[0]; ++x_idx ) {

            float normalized_depth = (texture_data[x_idx + y_idx * gbuffer_extraction_resolution_[0] ] );// / (float)(max_uint);
            printf("%.10f ", normalized_depth);
          }
          std::cout << "\n";
        }
      */
        // std::cout << "\n";

        std::string camera_uuid_string = "";
        auto const camera_view_id = camera.config.view_id();
        if(camera.config.enable_stereo())
        {
            bool is_left_cam = true;
            if(camera_view_id == last_rendered_view_id)
            {
                last_rendered_side = (last_rendered_side + 1) % 2;
                is_left_cam = (last_rendered_side == 0) ? true : false;
            }
            // std::cout << "Camera ID: " << camera.config.view_id() << (is_left_cam ? "L" : "R")<< "\n";

            camera_uuid_string = std::to_string(camera.config.view_id()) + (is_left_cam ? "L" : "R");
        }
        else
        {
            last_rendered_side = 0;
            camera_uuid_string = std::to_string(camera.config.view_id()) + "M";
        }

        last_rendered_view_id = camera_view_id;

        // std::cout << "\n";
        // std::cout << "Going to write: " << to_write.size() * 4 << " bytes\n";

        std::string const depth_buffer_object = "DB_" + camera_uuid_string;

        // std::cout << "DEPTH BUFFER OBJECT: " << depth_buffer_object << "\n";

        std::string const memory_segment_label_prefix = "segment_for_" + depth_buffer_object;

        auto memory_controller = gua::NamedSharedMemoryController::instance_shared_ptr();

        // std::cout << "Before trying to lock the mutex\n";
        memory_controller->lock_read_write();
        // std::cout << "After acquiring the mutex lock\n";

        memory_controller->add_read_only_memory_segment(memory_segment_label_prefix, false);
        memory_controller->register_remotely_constructed_object_on_segment(memory_segment_label_prefix, depth_buffer_object);

        //memory_controller->memcpy_buffer_to_named_object<std::array<char, gua::MemAllocSizes::KB64>>(depth_buffer_object.c_str(), (char*)&texture_data[0], texture_data.size() * 4);
        memory_controller->memcpy_buffer_to_named_object<std::array<char, gua::MemAllocSizes::MB1>>(depth_buffer_object.c_str(), (char*)&texture_data[0], texture_data.size() * 4);
        memory_controller->unlock_read_write();
        // std::cout << "After unlocking the mutex\n";
    };
#endif
    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}
} // namespace gua
