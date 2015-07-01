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

namespace gua {

void ABuffer::allocate(Pipeline& pipe, size_t buffer_size, math::vec2ui const& resolution) {

  auto& ctx(pipe.get_context());
  pipe_ = &pipe;

  if (!buffer_size) {
    res_ = nullptr;
    return;
  }

  // get a per-context resource
  res_ = ctx.resources.get<SharedResource>();

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

  res_->max_depth = ctx.render_device->create_texture_2d(resolution, scm::gl::data_format::FORMAT_R_32I);
  res_->min_depth = ctx.render_device->create_texture_2d(resolution, scm::gl::data_format::FORMAT_R_32I);

  // if (res_->min_max_buffer) {
  //   res_->min_max_buffer->make_non_resident(ctx);
  // }

  // int mip_map_levels = 5;
  // math::vec2 size(resolution/2);
  // scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_NEAREST,
  //   scm::gl::WRAP_CLAMP_TO_EDGE,
  //   scm::gl::WRAP_CLAMP_TO_EDGE);

  // res_->min_max_buffer = std::make_shared<Texture2D>(size.x, size.y,
  //   scm::gl::FORMAT_RG_32I, mip_map_levels, state);

  // res_->min_max_buffer_fbos.clear();

  // for (int i(0); i<mip_map_levels; ++i) {
  //   res_->min_max_buffer_fbos.push_back(ctx.render_device->create_frame_buffer());
  //   res_->min_max_buffer_fbos.back()->attach_color_buffer(0, res_->min_max_buffer->get_buffer(ctx),i,0);
  // }

}

ABuffer::~ABuffer() {
  // if (res_ && res_->min_max_buffer) {
  //   res_->min_max_buffer->make_non_resident(pipe_->get_context());
  // }
}

void ABuffer::allocate_shared(RenderContext const& with_ctx) {
  res_ = with_ctx.resources.get_dont_create<SharedResource>();
}

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

void ABuffer::update_min_max_buffer() {

  // auto& ctx(pipe_->get_context());
  // std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe_->current_viewstate().viewpoint_uuid) + " / ABuffer MinMaxMap";
  // pipe_->begin_gpu_query(ctx, gpu_query_name);


  // auto gbuffer = dynamic_cast<GBuffer*>(pipe_->current_viewstate().target);

  // // surface_detection_program_->use(ctx);
  // // surface_detection_program_->set_uniform(ctx, gbuffer->get_depth_buffer_write()->get_handle(ctx), "depth_buffer");
  // // surface_detection_program_->set_uniform(ctx, res_->surface_detection_buffer->get_handle(ctx), "surface_detection_buffer");

  // // for (int i(0); i<res_->surface_detection_buffer_fbos.size(); ++i) {
  // //   math::vec2ui level_size(scm::gl::util::mip_level_dimensions(resolution/2, i));
  // //   ctx.render_context->set_frame_buffer(res_->surface_detection_buffer_fbos[i]);
  // //   ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2f(0, 0), scm::math::vec2f(level_size)));
  // //   surface_detection_program_->set_uniform(ctx, i, "current_level");
  // //   pipe_->draw_quad();
  // // }

  // pipe_->end_gpu_query(ctx, gpu_query_name);
}

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

void ABuffer::unbind(RenderContext const& ctx) {}

}
