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
#include <gua/renderer/LineStripResource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/node/LineStripNode.hpp>
#include <gua/utils/Logger.hpp>

#include <scm/gl_core/constants.h>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

LineStripResource::LineStripResource() : kd_tree_(), line_strip_(), clean_flags_per_context_() { compute_bounding_box(); }

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::compute_bounding_box()
{
    // if (line_strip_.num_occupied_vertex_slots > 0) {
    bounding_box_ = math::BoundingBox<math::vec3>();

    if(0 == line_strip_.num_occupied_vertex_slots)
    {
        bounding_box_.expandBy(math::vec3{-0.5, -0.5, -0.5});
        bounding_box_.expandBy(math::vec3{0.5, 0.5, 0.5});
    }
    if(1 == line_strip_.num_occupied_vertex_slots)
    {
        bounding_box_.expandBy(math::vec3{line_strip_.positions[0] - 0.0001f});
        bounding_box_.expandBy(math::vec3{line_strip_.positions[0] + 0.0001f});
    }
    else
    {
        for(int v(0); v < line_strip_.num_occupied_vertex_slots; ++v)
        {
            bounding_box_.expandBy(math::vec3{line_strip_.positions[v]});
        }
    }
    //}
}

////////////////////////////////////////////////////////////////////////////////

LineStripResource::LineStripResource(LineStrip const& line_strip, bool build_kd_tree) : kd_tree_(), line_strip_(line_strip), clean_flags_per_context_()
{
    compute_bounding_box();

    if(build_kd_tree)
    {
        // kd_tree_.generate(line_strip);
    }
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::upload_to(RenderContext& ctx) const
{
    std::lock_guard<std::mutex> lock(line_strip_update_mutex_);

    /*
      if (line_strip_.vertex_reservoir_size == 0) {
        Logger::LOG_WARNING << "Unable to load LineStrip! Has no vertex data." << std::endl;
        return;
      }
    */

    auto line_strip_iterator = ctx.line_strips.find(uuid());

    bool update_cached_linestrip{false};

    if((ctx.line_strips.end() == line_strip_iterator))
    {
        ctx.line_strips[uuid()] = RenderContext::LineStrip();
    }

    RenderContext::LineStrip* line_strip_to_update_ptr = &ctx.line_strips[uuid()];

    if(update_cached_linestrip)
    {
        line_strip_to_update_ptr = &(line_strip_iterator->second);
    }

    line_strip_to_update_ptr->vertex_topology = scm::gl::PRIMITIVE_LINE_STRIP_ADJACENCY;
    line_strip_to_update_ptr->vertex_reservoir_size = line_strip_.vertex_reservoir_size;
    line_strip_to_update_ptr->num_occupied_vertex_slots = line_strip_.num_occupied_vertex_slots;

    if(line_strip_to_update_ptr->current_buffer_size_in_vertices < line_strip_.vertex_reservoir_size + 3)
    {
        line_strip_to_update_ptr->vertices =
            ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_DYNAMIC_DRAW, (line_strip_.vertex_reservoir_size + 3) * sizeof(LineStrip::Vertex), 0);

        line_strip_to_update_ptr->current_buffer_size_in_vertices = line_strip_.vertex_reservoir_size + 3;
    }
    else
    {
        update_cached_linestrip = true;
    }

    if(line_strip_.vertex_reservoir_size != 0)
    {
        LineStrip::Vertex* data(static_cast<LineStrip::Vertex*>(ctx.render_context->map_buffer(line_strip_to_update_ptr->vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

        line_strip_.copy_to_buffer(data);
        ctx.render_context->unmap_buffer(line_strip_to_update_ptr->vertices);

        line_strip_to_update_ptr->vertex_array = ctx.render_device->create_vertex_array(line_strip_.get_vertex_format(), {line_strip_to_update_ptr->vertices});
    }

    ctx.render_context->apply();
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::draw(RenderContext& ctx) const
{
    // DUMMY
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::draw(RenderContext& ctx, bool render_vertices_as_points, bool render_lines_as_strip) const
{
    auto iter = ctx.line_strips.find(uuid());

    bool& clean_flag_for_context = clean_flags_per_context_[uuid()];

    if(iter == ctx.line_strips.end() || (!clean_flag_for_context) /*|| ctx_dirty_flag*/)
    {
        // upload to GPU if neccessary

        compute_consistent_normals();
        upload_to(ctx);
        iter = ctx.line_strips.find(uuid());

        clean_flag_for_context = true;
        // dirty_flags_per_context_[uuid()] = false;;
    }

    ctx.render_context->bind_vertex_array(iter->second.vertex_array);
    // ctx.render_context->bind_index_buffer(iter->second.indices, iter->second.indices_topology, iter->second.indices_type);
    ctx.render_context->apply_vertex_input();

    if(!render_vertices_as_points)
    {
        // ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_LINE_LOOP, 0, iter->second.num_occupied_vertex_slots+2);
        if(render_lines_as_strip)
        {
            ctx.render_context->draw_arrays(iter->second.vertex_topology, 0, iter->second.num_occupied_vertex_slots + 3);
        }
        else
        {
            ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_LINE_LIST, 1, iter->second.num_occupied_vertex_slots);
        }
    }
    else
    {
        ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 1, iter->second.num_occupied_vertex_slots);
    }
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits)
{
    // kd_tree_.ray_test(ray, line_strip_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////
/*
void LineStripResource::resolve_vertex_updates(RenderContext& ctx) {

  //TODO: PROTECT BY LINE STRIP UPDATE MUTEX

  for(auto& update_job_ptr : line_strip_update_queue_) {
    auto iter = ctx.line_strips.find(update_job_ptr->owner_uuid);
    auto ctx_dirty_flag_iter = dirty_flags_per_context_.find(uuid());
    bool ctx_dirty_flag = false;

    if (iter == ctx.line_strips.end() || ctx_dirty_flag) {
      dirty_flags_per_context_[uuid()] = false;;
    }

    update_job_ptr->execute(this);
  }
}
*/
////////////////////////////////////////////////////////////////////////////////

void LineStripResource::make_clean_flags_dirty()
{
    for(auto& known_clean_flag : clean_flags_per_context_)
    {
        known_clean_flag.second = false;
    }
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::compute_consistent_normals() const
{
    std::lock_guard<std::mutex> lock(line_strip_update_mutex_);
    line_strip_.compute_consistent_normals();
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::compile_buffer_string(std::string& buffer_string)
{
    std::lock_guard<std::mutex> lock(line_strip_update_mutex_);
    line_strip_.compile_buffer_string(buffer_string);
};

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::uncompile_buffer_string(std::string const& buffer_string)
{
    std::lock_guard<std::mutex> lock(line_strip_update_mutex_);
    line_strip_.uncompile_buffer_string(buffer_string);
    make_clean_flags_dirty();
};

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::push_vertex(LineStrip::Vertex const& in_vertex)
{
    std::lock_guard<std::mutex> lock(line_strip_update_mutex_);

    if(line_strip_.push_vertex(in_vertex))
    {
        if(line_strip_.num_occupied_vertex_slots > 0)
        {
            bounding_box_.expandBy(math::vec3{line_strip_.positions[line_strip_.num_occupied_vertex_slots - 1]});
        }
        make_clean_flags_dirty();
    }
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::pop_front_vertex()
{
    std::lock_guard<std::mutex> lock(line_strip_update_mutex_);
    if(line_strip_.pop_front_vertex())
    {
        compute_bounding_box();
        make_clean_flags_dirty();
    }
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::pop_back_vertex()
{
    std::lock_guard<std::mutex> lock(line_strip_update_mutex_);
    if(line_strip_.pop_back_vertex())
    {
        compute_bounding_box();
        make_clean_flags_dirty();
    }
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::clear_vertices()
{
    std::lock_guard<std::mutex> lock(line_strip_update_mutex_);
    if(line_strip_.clear_vertices())
    {
        compute_bounding_box();
        make_clean_flags_dirty();
    }
}

////////////////////////////////////////////////////////////////////////////////

void LineStripResource::forward_queued_vertices(std::vector<scm::math::vec3f> const& queued_positions,
                                                std::vector<scm::math::vec4f> const& queued_colors,
                                                std::vector<float> const& queued_thicknesses,
                                                std::vector<scm::math::vec3f> const& queued_normals)
{
    std::lock_guard<std::mutex> lock(line_strip_update_mutex_);
    line_strip_.forward_queued_vertices(queued_positions, queued_colors, queued_thicknesses, queued_normals);
    compute_bounding_box();
    make_clean_flags_dirty();
}

////////////////////////////////////////////////////////////////////////////////

math::vec3 LineStripResource::get_vertex(unsigned int i) const { return math::vec3(line_strip_.positions[i].x, line_strip_.positions[i].y, line_strip_.positions[i].z); }

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
