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
#include <gua/renderer/WarpGridGenerator.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/GenerateWarpGridPass.hpp>
#include <gua/databases/Resources.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

WarpGridGenerator::WarpGridGenerator()
{
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  ResourceFactory factory;
  std::string v_shader = factory.read_shader_file("shaders/warp_grid_generate_shader.vert");
  std::string g_shader = factory.read_shader_file("shaders/warp_grid_generate_shader.geom");
#else
  std::string v_shader = Resources::lookup_shader("shaders/warp_grid_generate_shader.vert");
  std::string g_shader = Resources::lookup_shader("shaders/warp_grid_generate_shader.geom");
#endif

  grid_generation_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
  grid_generation_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, g_shader));


#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  v_shader = factory.read_shader_file("shaders/common/fullscreen_quad.vert");
  g_shader = factory.read_shader_file("shaders/min_max_filter_shader.frag");
#else
  v_shader = Resources::lookup_shader("shaders/common/fullscreen_quad.vert");
  g_shader = Resources::lookup_shader("shaders/min_max_filter_shader.frag");
#endif

  min_max_filter_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
  min_max_filter_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, g_shader));
}

////////////////////////////////////////////////////////////////////////////////

WarpGridGenerator::~WarpGridGenerator() {
  if (res_ && res_->min_max_depth_buffer) {
    res_->min_max_depth_buffer->make_non_resident(pipe_->get_context());
  }
}

////////////////////////////////////////////////////////////////////////////////

void WarpGridGenerator::render(Pipeline& pipe, PipelinePassDescription const& desc)
{
  auto& ctx(pipe.get_context());

  pipe_ = &pipe;

  std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpGridGenerator";
  std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpGridGenerator";
  std::string const pri_query_name = "Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / WarpGridGenerator";

  pipe.begin_primitive_query(ctx, pri_query_name);
  pipe.begin_gpu_query(ctx, gpu_query_name);
  pipe.begin_cpu_query(cpu_query_name);

  math::vec2ui resolution(pipe.current_viewstate().camera.config.get_resolution());
  size_t pixel_count(resolution.x * resolution.y);

  if (!grid_generation_program_) {
    grid_generation_program_ = std::make_shared<ShaderProgram>();
    grid_generation_program_->set_shaders(grid_generation_program_stages_, {"xfb_output"}, true, global_substitution_map_);
  }

  if (!min_max_filter_program_) {
    min_max_filter_program_ = std::make_shared<ShaderProgram>();
    min_max_filter_program_->set_shaders(min_max_filter_program_stages_, {}, false, global_substitution_map_);
  }

  // create resources
  if (!res_) {
    res_ = ctx.resources.get<SharedResource>();

    auto description(dynamic_cast<GenerateWarpGridPassDescription const*>(&desc));

    if (!res_->grid_vbo[0] || res_->cell_count < pixel_count) {
      for (int i(0); i<2; ++i) {
        res_->grid_vbo[i] =
          ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                           scm::gl::USAGE_DYNAMIC_DRAW,
                                           pixel_count * sizeof(math::vec3i));
        res_->grid_vao[i] = ctx.render_device->create_vertex_array(
          scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3I, sizeof(math::vec3i)), {res_->grid_vbo[i]});
        res_->grid_tfb[i] = ctx.render_device->create_transform_feedback(
          scm::gl::stream_output_setup(res_->grid_vbo[i]));
      }
      res_->cell_count = pixel_count;


      if (res_->min_max_depth_buffer) {
        res_->min_max_depth_buffer->make_non_resident(ctx);
      }

      math::vec2ui size(resolution/2);
      unsigned mip_map_levels(scm::gl::util::max_mip_levels(size/32.f));
      scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_NEAREST,
        scm::gl::WRAP_CLAMP_TO_EDGE,
        scm::gl::WRAP_CLAMP_TO_EDGE);

      if (description->mode() == GenerateWarpGridPassDescription::DEPTH_THRESHOLD) {
        res_->min_max_depth_buffer = std::make_shared<Texture2D>(size.x, size.y,
            scm::gl::FORMAT_RGB_16, mip_map_levels, state);
      } else {
        res_->min_max_depth_buffer = std::make_shared<Texture2D>(size.x, size.y,
            scm::gl::FORMAT_R_8I, mip_map_levels, state);
      }

      res_->min_max_depth_buffer_fbos.clear();

      for (int i(0); i<mip_map_levels; ++i) {
        res_->min_max_depth_buffer_fbos.push_back(ctx.render_device->create_frame_buffer());
        res_->min_max_depth_buffer_fbos.back()->attach_color_buffer(0, res_->min_max_depth_buffer->get_buffer(ctx),i,0);
      }
    }
  }

  // ---------------------------------------------------------------------------
  // ---------------------- MinMax Depth Map -----------------------------------
  // ---------------------------------------------------------------------------

  auto gbuffer = dynamic_cast<GBuffer*>(pipe.current_viewstate().target);

  min_max_filter_program_->use(ctx);
  min_max_filter_program_->set_uniform(ctx, gbuffer->get_depth_buffer_write()->get_handle(ctx), "depth_buffer");
  min_max_filter_program_->set_uniform(ctx, res_->min_max_depth_buffer->get_handle(ctx), "min_max_depth_buffer");

  for (int i(0); i<res_->min_max_depth_buffer_fbos.size(); ++i) {
    math::vec2ui level_size(scm::gl::util::mip_level_dimensions(resolution/2, i));
    ctx.render_context->set_frame_buffer(res_->min_max_depth_buffer_fbos[i]);
    ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2f(0, 0), scm::math::vec2f(level_size)));
    min_max_filter_program_->set_uniform(ctx, i, "current_level");
    pipe.draw_quad();
  }

  // ---------------------------------------------------------------------------
  // --------------------- Generate Warp Grid ----------------------------------
  // ---------------------------------------------------------------------------

  uint initial_grid_x(std::ceil(resolution.x/32.f));
  uint initial_grid_y(std::ceil(resolution.y/32.f));

  {
    // upload initial data
    auto data = static_cast<math::vec3i*>(ctx.render_context->map_buffer(
        res_->grid_vbo[res_->current_vbo()], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER));

    for (int x(0); x < initial_grid_x; ++x) {
      for (int y(0); y < initial_grid_y; ++y) {
        data[y*initial_grid_x + x] = math::vec3i(x*32, y*32, 32);
      }
    }

    ctx.render_context->unmap_buffer(res_->grid_vbo[res_->current_vbo()]);
  }

  grid_generation_program_->use(ctx);
  grid_generation_program_->set_uniform(ctx, res_->min_max_depth_buffer->get_handle(ctx), "min_max_depth_buffer");
  grid_generation_program_->set_uniform(ctx, 32, "current_cellsize");

  // first subdivision
  ctx.render_context->begin_transform_feedback(res_->grid_tfb[res_->current_tfb()],
    scm::gl::PRIMITIVE_POINTS);
  {
    ctx.render_context->bind_vertex_array(res_->grid_vao[res_->current_vbo()]);
    ctx.render_context->apply();
    ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, initial_grid_x*initial_grid_y);
  }

  ctx.render_context->end_transform_feedback();
  res_->ping = !res_->ping;

  for (int i(0); i<4; ++i) {
    // further subdivisions
    grid_generation_program_->set_uniform(ctx, (int)std::pow(2, 4-i), "current_cellsize");
    ctx.render_context->begin_transform_feedback(res_->grid_tfb[res_->current_tfb()],
      scm::gl::PRIMITIVE_POINTS);
    {
      ctx.render_context->bind_vertex_array(res_->grid_vao[res_->current_vbo()]);
      ctx.render_context->apply();
      ctx.render_context->draw_transform_feedback(scm::gl::PRIMITIVE_POINT_LIST, res_->grid_tfb[res_->current_vbo()]);
    }

    ctx.render_context->end_transform_feedback();
    res_->ping = !res_->ping;
  }

  pipe.end_primitive_query(ctx, pri_query_name);
  pipe.end_gpu_query(ctx, gpu_query_name);
  pipe.end_cpu_query(cpu_query_name);

  ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
