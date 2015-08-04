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
#include <gua/renderer/ABuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/Resources.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

ABuffer::ABuffer()
  : pipe_(nullptr) {}

////////////////////////////////////////////////////////////////////////////////

void ABuffer::allocate(Pipeline& pipe, size_t buffer_size, math::vec2ui const& resolution) {

  auto& ctx(pipe.get_context());
  pipe_ = &pipe;

  // get a per-context resource
  res_ = ctx.resources.get<SharedResource>();

  if (res_->min_max_buffer) {
    res_->min_max_buffer->make_non_resident(ctx);
  }

  if (!buffer_size) {
    res_ = nullptr;
    return;
  }


  // compute memory allowance
  size_t frag_count = (buffer_size * 1024u * 1024u)
                      / (FRAG_LIST_WORD_SIZE + FRAG_DATA_WORD_SIZE);

  // init/reinit if necessary
  if (!res_->counter || res_->frag_count < frag_count) {

    res_->frag_count = frag_count;

    res_->counter =
        ctx.render_device->create_buffer(scm::gl::BIND_ATOMIC_COUNTER_BUFFER,
                                         scm::gl::USAGE_DYNAMIC_COPY,
                                         sizeof(unsigned));
    res_->frag_list =
        ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER,
                                         scm::gl::USAGE_DYNAMIC_COPY,
                                         frag_count * FRAG_LIST_WORD_SIZE);
    res_->frag_data =
        ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER,
                                         scm::gl::USAGE_DYNAMIC_COPY,
                                         frag_count * FRAG_DATA_WORD_SIZE);

  }

  res_->max_depth = ctx.render_device->create_texture_2d(resolution/2, scm::gl::data_format::FORMAT_R_32UI);
  res_->min_depth = ctx.render_device->create_texture_2d(resolution/2, scm::gl::data_format::FORMAT_R_32UI);

  math::vec2 size(resolution/2);
  int mip_map_levels = scm::gl::util::max_mip_levels(math::vec2ui(size));
  scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_NEAREST,
    scm::gl::WRAP_CLAMP_TO_EDGE,
    scm::gl::WRAP_CLAMP_TO_EDGE);
  res_->min_max_buffer = std::make_shared<Texture2D>(size.x, size.y,
    scm::gl::FORMAT_RG_32UI, mip_map_levels, state);

  res_->min_max_buffer_fbos.clear();

  for (int i(0); i<mip_map_levels; ++i) {
    res_->min_max_buffer_fbos.push_back(ctx.render_device->create_frame_buffer());
    res_->min_max_buffer_fbos.back()->attach_color_buffer(0, res_->min_max_buffer->get_buffer(ctx),i,0);
  }

  #ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    std::string v_shader = factory.read_shader_file("shaders/common/fullscreen_quad.vert");
    std::string f_shader = factory.read_shader_file("shaders/abuf_min_max.frag");
  #else
    std::string v_shader = Resources::lookup_shader("shaders/common/fullscreen_quad.vert");
    std::string f_shader = Resources::lookup_shader("shaders/abuf_min_max.frag");
  #endif

  min_max_program_ = std::make_shared<ShaderProgram>();
  min_max_program_->create_from_sources(v_shader, f_shader);
}

////////////////////////////////////////////////////////////////////////////////

ABuffer::~ABuffer() {
  if (res_ && res_->min_max_buffer) {
    res_->min_max_buffer->make_non_resident(pipe_->get_context());
  }
}

////////////////////////////////////////////////////////////////////////////////

void ABuffer::clear(RenderContext const& ctx, math::vec2ui const& resolution) {

  if (!res_) {
    return;
  }

  unsigned* ctr = reinterpret_cast<unsigned*>(ctx.render_context->map_buffer(
      res_->counter, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER));
  if (ctr) {
    *ctr = 0;
  }
  ctx.render_context->unmap_buffer(res_->counter);

  ctx.render_context->clear_buffer_sub_data(res_->frag_list,
                                            scm::gl::FORMAT_RG_32UI,
                                            0u,
                                            FRAG_LIST_WORD_SIZE * resolution.x
                                                                * resolution.y,
                                            0);
  ctx.render_context->clear_image_data(res_->min_depth, 0, scm::gl::data_format::FORMAT_R_32I, 0);
  ctx.render_context->clear_image_data(res_->max_depth, 0, scm::gl::data_format::FORMAT_R_32I, 0);
}

////////////////////////////////////////////////////////////////////////////////

void ABuffer::update_min_max_buffer() {

  if (res_) {
    auto& ctx(pipe_->get_context());
    std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe_->current_viewstate().viewpoint_uuid) + " / ABuffer MinMaxMap";
    pipe_->begin_gpu_query(ctx, gpu_query_name);

    bind(ctx);

    min_max_program_->use(ctx);

    bind_min_max_buffer(min_max_program_);

    for (int i(0); i<res_->min_max_buffer_fbos.size(); ++i) {
      math::vec2ui level_size(scm::gl::util::mip_level_dimensions(math::vec2ui(res_->min_max_buffer->width(), res_->min_max_buffer->height()), i));
      ctx.render_context->set_frame_buffer(res_->min_max_buffer_fbos[i]);
      ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2f(0, 0), scm::math::vec2f(level_size)));
      min_max_program_->set_uniform(ctx, i, "current_level");
      pipe_->draw_quad();
    }

    pipe_->end_gpu_query(ctx, gpu_query_name);
  }
}

////////////////////////////////////////////////////////////////////////////////

void ABuffer::bind_min_max_buffer(std::shared_ptr<ShaderProgram> const& shader) {
  if (pipe_ && res_ && res_->min_max_buffer) {
    shader->set_uniform(pipe_->get_context(), res_->min_max_buffer->get_handle(pipe_->get_context()), "abuf_min_max_depth");
  }
}

////////////////////////////////////////////////////////////////////////////////

void ABuffer::bind(RenderContext const& ctx) {

  if (!res_) {
    return;
  }

  ctx.render_context->bind_atomic_counter_buffer(res_->counter, 0);
  ctx.render_context->bind_storage_buffer(res_->frag_list, 0);
  ctx.render_context->bind_storage_buffer(res_->frag_data, 1);
  ctx.render_context->bind_image(res_->min_depth, 
                                 scm::gl::data_format::FORMAT_R_32I, 
                                 scm::gl::access_mode::ACCESS_READ_WRITE, 0);
  ctx.render_context->bind_image(res_->max_depth, 
                                 scm::gl::data_format::FORMAT_R_32I, 
                                 scm::gl::access_mode::ACCESS_READ_WRITE, 1);
}

////////////////////////////////////////////////////////////////////////////////

void ABuffer::unbind(RenderContext const& ctx) {}

////////////////////////////////////////////////////////////////////////////////

}

////////////////////////////////////////////////////////////////////////////////