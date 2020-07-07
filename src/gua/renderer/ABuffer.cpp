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

namespace gua
{
void ABuffer::allocate(RenderContext& ctx, size_t buffer_size)
{
    if(!buffer_size)
    {
        res_ = nullptr;
        return;
    }

    // get a per-context resource
    auto resource = ctx.resources.get<SharedResource>();

    // compute memory allowance
    size_t frag_count = (buffer_size * 1024u * 1024u) / (FRAG_LIST_WORD_SIZE + FRAG_DATA_WORD_SIZE);

    // init/reinit if necessary
    if(!resource->counter || resource->frag_count < frag_count)
    {
        resource->frag_count = frag_count;

        resource->counter = ctx.render_device->create_buffer(scm::gl::BIND_ATOMIC_COUNTER_BUFFER, scm::gl::USAGE_DYNAMIC_COPY, sizeof(unsigned));
        resource->frag_list = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_DYNAMIC_COPY, frag_count * FRAG_LIST_WORD_SIZE);
        resource->frag_data = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_DYNAMIC_COPY, frag_count * FRAG_DATA_WORD_SIZE);
    }
    res_ = resource;
}

void ABuffer::clear(RenderContext const& ctx, math::vec2ui const& resolution)
{
    if(!res_)
    {
        return;
    }

    unsigned* ctr = reinterpret_cast<unsigned*>(ctx.render_context->map_buffer(res_->counter, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER));
    if(ctr)
    {
        *ctr = 0;
    }
    ctx.render_context->unmap_buffer(res_->counter);

    ctx.render_context->clear_buffer_sub_data(res_->frag_list, scm::gl::FORMAT_RG_32UI, 0u, FRAG_LIST_WORD_SIZE * resolution.x * resolution.y, 0);
}

void ABuffer::bind(RenderContext const& ctx, uint offset)
{
    if(!res_)
    {
        return;
    }

    ctx.render_context->bind_atomic_counter_buffer(res_->counter, 0 + offset);
    ctx.render_context->bind_storage_buffer(res_->frag_list, 0 + 2 * offset);
    ctx.render_context->bind_storage_buffer(res_->frag_data, 1 + 2 * offset);
}

void ABuffer::unbind(RenderContext const& ctx) {}

} // namespace gua
