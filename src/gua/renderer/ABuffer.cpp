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

// guacamole headers
#include <gua/utils/Logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

void ABuffer::allocate(RenderContext const& ctx, size_t buffer_size) {

  if (!buffer_size) {
    res_ = nullptr;
    return;
  }

  // get a per-context resource
  auto resource = ctx.resources.get<SharedResource>();

  // compute memory allowance
  const size_t data_chunk_size = 2;
  size_t frag_list_size = (buffer_size * 1024u * 1024u) / (data_chunk_size + 1);
  frag_list_size = (frag_list_size / (sizeof(unsigned) * 4)) * (sizeof(unsigned) * 4);
  size_t frag_data_size = frag_list_size * data_chunk_size;

  // init/reinit if necessary
  if (!resource->counter 
      && resource->frag_list_size < frag_list_size) {

    Logger::LOG_MESSAGE << "Init ABuffer to hold " 
                        << frag_list_size << " fragments" << std::endl;
    resource->frag_list_size = frag_list_size;

    resource->counter   = ctx.render_device->create_buffer(scm::gl::BIND_ATOMIC_COUNTER_BUFFER, 
                                                           scm::gl::USAGE_DYNAMIC_COPY, 
                                                           sizeof(unsigned));
    resource->frag_list = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, 
                                                           scm::gl::USAGE_DYNAMIC_COPY, 
                                                           frag_list_size);
    resource->frag_data = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, 
                                                           scm::gl::USAGE_DYNAMIC_COPY, 
                                                           frag_data_size);
  }
  res_ = resource;
}

////////////////////////////////////////////////////////////////////////////////

void ABuffer::clear(RenderContext const& ctx, math::vec2ui const& resolution) {

  if (!res_) {
    return;
  }

  unsigned* ctr = reinterpret_cast<unsigned*>(
      ctx.render_context->map_buffer(res_->counter,
                                     scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER));
  if (ctr) { 
    *ctr = 0;
  }
  ctx.render_context->unmap_buffer(res_->counter);

  ctx.render_context->clear_buffer_sub_data(res_->frag_list, 
                                            scm::gl::FORMAT_RG_32UI, 
                                            0u, 
                                            8u * resolution.x * resolution.y,
                                            0);
}

////////////////////////////////////////////////////////////////////////////////

void ABuffer::bind(RenderContext const& ctx) {

  if (!res_) {
    return;
  }

  ctx.render_context->bind_atomic_counter_buffer(res_->counter, 0);
  ctx.render_context->bind_storage_buffer(res_->frag_list, 0);
  ctx.render_context->bind_storage_buffer(res_->frag_data, 1);
}

////////////////////////////////////////////////////////////////////////////////

void ABuffer::unbind(RenderContext const& ctx) {

}

////////////////////////////////////////////////////////////////////////////////

}
