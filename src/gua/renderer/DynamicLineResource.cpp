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
#include <gua/renderer/DynamicLineResource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/node/DynamicLineNode.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/KDTreeUtils.hpp>

#include <scm/gl_core/constants.h>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

DynamicLineResource::DynamicLineResource() : DynamicGeometryResource(scm::gl::PRIMITIVE_LINE_LIST)
{
    //: kd_tree_(), dynamic_line_(), vertex_rendering_mode_(scm::gl::PRIMITIVE_TRIANGLE_LIST), clean_flags_per_context_() {
    // : kd_tree_(), dynamic_line_(), vertex_rendering_mode_(scm::gl::PRIMITIVE_LINE_LIST), clean_flags_per_context_() {
    compute_bounding_box();
}

////////////////////////////////////////////////////////////////////////////////

DynamicLineResource::DynamicLineResource(std::shared_ptr<DynamicGeometry> dynamic_geometry_ptr, bool build_kd_tree)
    : DynamicGeometryResource(dynamic_geometry_ptr, build_kd_tree, scm::gl::PRIMITIVE_LINE_LIST)
{
    compute_bounding_box();

    if(build_kd_tree)
    {
        // kd_tree_.generate(dynamic_line);
    }
}

////////////////////////////////////////////////////////////////////////////////

void DynamicLineResource::compute_bounding_box()
{
    // if (dynamic_line_.num_occupied_vertex_slots > 0) {
    bounding_box_ = math::BoundingBox<math::vec3>();

    if(0 == dynamic_geometry_ptr_->num_occupied_vertex_slots)
    {
        bounding_box_.expandBy(math::vec3{-0.5, -0.5, -0.5});
        bounding_box_.expandBy(math::vec3{0.5, 0.5, 0.5});
    }
    if(1 == dynamic_geometry_ptr_->num_occupied_vertex_slots)
    {
        bounding_box_.expandBy(math::vec3{dynamic_geometry_ptr_->positions[0] - 0.0001f});
        bounding_box_.expandBy(math::vec3{dynamic_geometry_ptr_->positions[0] + 0.0001f});
    }
    else
    {
        for(int v(0); v < dynamic_geometry_ptr_->num_occupied_vertex_slots; ++v)
        {
            bounding_box_.expandBy(math::vec3{dynamic_geometry_ptr_->positions[v]});
        }
    }
    //}
}

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

void DynamicLineResource::upload_to(RenderContext &ctx) const
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
    /*
      if (dynamic_line_.vertex_reservoir_size == 0) {
        Logger::LOG_WARNING << "Unable to load DynamicLine! Has no vertex data." << std::endl;
        return;
      }
    */
    auto dynamic_line_iterator = ctx.dynamic_geometries.find(uuid());

    bool update_cached_dynamic_line{false};

    if((ctx.dynamic_geometries.end() == dynamic_line_iterator))
    {
        ctx.dynamic_geometries[uuid()] = RenderContext::DynamicGeometry();
    }

    RenderContext::DynamicGeometry *dynamic_line_to_update_ptr = &ctx.dynamic_geometries[uuid()];

    if(update_cached_dynamic_line)
    {
        dynamic_line_to_update_ptr = &(dynamic_line_iterator->second);
    }

    dynamic_line_to_update_ptr->vertex_topology = scm::gl::PRIMITIVE_LINE_LIST;
    dynamic_line_to_update_ptr->vertex_reservoir_size = dynamic_geometry_ptr_->vertex_reservoir_size;
    dynamic_line_to_update_ptr->num_occupied_vertex_slots = dynamic_geometry_ptr_->num_occupied_vertex_slots;

    if(dynamic_line_to_update_ptr->current_buffer_size_in_vertices < dynamic_geometry_ptr_->vertex_reservoir_size)
    {
        dynamic_line_to_update_ptr->vertices =
            ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_DYNAMIC_DRAW, (dynamic_geometry_ptr_->vertex_reservoir_size) * sizeof(DynamicGeometry::Vertex), 0);

        dynamic_line_to_update_ptr->current_buffer_size_in_vertices = dynamic_geometry_ptr_->vertex_reservoir_size + 3;
    }
    else
    {
        update_cached_dynamic_line = true;
    }

    if(dynamic_geometry_ptr_->vertex_reservoir_size != 0)
    {
        DynamicGeometry::Vertex *data(static_cast<DynamicGeometry::Vertex *>(ctx.render_context->map_buffer(dynamic_line_to_update_ptr->vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

        std::dynamic_pointer_cast<DynamicGeometry>(dynamic_geometry_ptr_)->copy_to_buffer(data);
        ctx.render_context->unmap_buffer(dynamic_line_to_update_ptr->vertices);

        dynamic_line_to_update_ptr->vertex_array = ctx.render_device->create_vertex_array(dynamic_geometry_ptr_->get_vertex_format(), {dynamic_line_to_update_ptr->vertices});
    }

    ctx.render_context->apply();
}

////////////////////////////////////////////////////////////////////////////////

void DynamicLineResource::draw(RenderContext &ctx) const
{
    int triangles = num_occupied_vertex_slots() / 3;
    // std::cout<< "triangles " << triangles << std::endl;

    auto iter = ctx.dynamic_geometries.find(uuid());

    bool &clean_flag_for_context = clean_flags_per_context_[uuid()];

    if(iter == ctx.dynamic_geometries.end() || (!clean_flag_for_context) /*|| ctx_dirty_flag*/)
    {
        // upload to GPU if neccessary

        compute_consistent_normals();
        upload_to(ctx);
        iter = ctx.dynamic_geometries.find(uuid());

        clean_flag_for_context = true;
        // dirty_flags_per_context_[uuid()] = false;;
    }

    ctx.render_context->bind_vertex_array(iter->second.vertex_array);
    // ctx.render_context->bind_index_buffer(iter->second.indices, iter->second.indices_topology, iter->second.indices_type);
    ctx.render_context->apply_vertex_input();

    ctx.render_context->draw_arrays(iter->second.vertex_topology, 0, iter->second.num_occupied_vertex_slots);
}

////////////////////////////////////////////////////////////////////////////////

void DynamicLineResource::ray_test(Ray const &ray, int options, node::Node *owner, std::set<PickResult> &hits)
{
    std::vector<scm::math::vec3f> vertex_positions;
    {
        std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
        vertex_positions = dynamic_geometry_ptr_->positions;
    }
    int triangles = num_occupied_vertex_slots() / 3;

    float min_intersection = std::numeric_limits<float>::max();
    bool intersected(false);

    std::array<math::vec3, 3> tri_vertices;
    //#pragma omp parallel for reduction(min:min_intersection)
    for(int tri_idx = 0; tri_idx < triangles; ++tri_idx)
    {
        std::size_t tri_base_offset = tri_idx * 3;
        tri_vertices[0] = vertex_positions[tri_base_offset];
        tri_vertices[1] = vertex_positions[tri_base_offset + 1];
        tri_vertices[2] = vertex_positions[tri_base_offset + 2];

        float current_intersection = intersect(tri_vertices, ray);
        min_intersection = std::min(min_intersection, current_intersection);
    }

    if(min_intersection < Ray::END)
    {
        if(hits.empty() || min_intersection < hits.begin()->distance)
        {
            hits.clear();
            float const inf(std::numeric_limits<float>::max());
            math::vec3 position(inf, inf, inf), world_position(inf, inf, inf), normal(inf, inf, inf), world_normal(inf, inf, inf);
            math::vec2 tex_coords;

            if(options & PickResult::GET_POSITIONS || options & PickResult::GET_WORLD_POSITIONS || options & PickResult::INTERPOLATE_NORMALS || options & PickResult::GET_TEXTURE_COORDS)
            {
                position = ray.origin_ + min_intersection * ray.direction_;
            }

            hits.insert(PickResult(min_intersection, owner, position, world_position, normal, world_normal, tex_coords));
            intersected = true;
        }
    }
}

float DynamicLineResource::intersect(std::array<math::vec3, 3> const &points, Ray const &ray) const
{
    // MOELLER TRUMBORE
    // Find Line Normal
    math::vec3 normal = scm::math::cross(points[1] - points[0], points[2] - points[0]);
    scm::math::normalize(normal);

    // Find distance from LP1 and LP2 to the plane defined by the triangle
    float dist1 = scm::math::dot(ray.origin_ - points[0], normal);
    float dist2 = scm::math::dot(ray.origin_ + ray.direction_ - points[0], normal);

    if((dist1 * dist2) >= 0.0f)
    {
        return Ray::END;
    } // line doesn't cross the triangle.

    if(dist1 == dist2)
    {
        return Ray::END;
    } // line and plane are parallel

    // Find point on the line that intersects with the plane
    float t = -dist1 / (dist2 - dist1);

    if(t > ray.t_max_)
    {
        return Ray::END;
    } // intersection is too far away

    if(t < 0.0f)
    {
        return Ray::END;
    }

    math::vec3 intersection = ray.origin_ + (ray.direction_) * t;

    // Find if the interesection point lies inside the triangle by testing it
    // against all edges
    math::vec3 test = scm::math::cross(normal, points[1] - points[0]);
    if(scm::math::dot(test, intersection - points[0]) < 0.0f)
    {
        return Ray::END;
    }

    test = scm::math::cross(normal, points[2] - points[1]);
    if(scm::math::dot(test, intersection - points[1]) < 0.0f)
    {
        return Ray::END;
    }

    test = scm::math::cross(normal, points[0] - points[2]);
    if(scm::math::dot(test, intersection - points[0]) < 0.0f)
    {
        return Ray::END;
    }

    return t;
}

////////////////////////////////////////////////////////////////////////////////
/*
void DynamicLineResource::resolve_vertex_updates(RenderContext& ctx) {

  //TODO: PROTECT BY LINE STRIP UPDATE MUTEX

  for(auto& update_job_ptr : dynamic_line_update_queue_) {
    auto iter = ctx.dynamic_geometries.find(update_job_ptr->owner_uuid);
    auto ctx_dirty_flag_iter = dirty_flags_per_context_.find(uuid());
    bool ctx_dirty_flag = false;

    if (iter == ctx.dynamic_geometries.end() || ctx_dirty_flag) {
      dirty_flags_per_context_[uuid()] = false;;
    }

    update_job_ptr->execute(this);
  }
}
*/
////////////////////////////////////////////////////////////////////////////////

void DynamicLineResource::make_clean_flags_dirty()
{
    for(auto &known_clean_flag : clean_flags_per_context_)
    {
        known_clean_flag.second = false;
    }
}

////////////////////////////////////////////////////////////////////////////////

void DynamicLineResource::compute_consistent_normals() const
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
    dynamic_geometry_ptr_->compute_consistent_normals();
}

////////////////////////////////////////////////////////////////////////////////

void DynamicLineResource::uncompile_buffer_string(std::string const &buffer_string)
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
    dynamic_geometry_ptr_->uncompile_buffer_string(buffer_string);
    make_clean_flags_dirty();
};

////////////////////////////////////////////////////////////////////////////////


void DynamicLineResource::compile_buffer_string(std::string &buffer_string)
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
    dynamic_geometry_ptr_->compile_buffer_string(buffer_string);
}


////////////////////////////////////////////////////////////////////////////////

void DynamicLineResource::push_vertex(DynamicGeometry::Vertex const &in_vertex)
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);

    if(std::dynamic_pointer_cast<DynamicGeometry>(dynamic_geometry_ptr_)->push_vertex(in_vertex))
    {
        if(dynamic_geometry_ptr_->num_occupied_vertex_slots > 0)
        {
            bounding_box_.expandBy(math::vec3{dynamic_geometry_ptr_->positions[dynamic_geometry_ptr_->num_occupied_vertex_slots - 1]});
        }
        make_clean_flags_dirty();
    }
}

void DynamicLineResource::update_vertex(int vertex_idx, DynamicGeometry::Vertex const &in_vertex)
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);

    if(std::dynamic_pointer_cast<DynamicGeometry>(dynamic_geometry_ptr_)->update_vertex(vertex_idx, in_vertex))
    {
        if(dynamic_geometry_ptr_->num_occupied_vertex_slots > 0)
        {
            // TODO remove old vertex position
        }
        make_clean_flags_dirty();
    }
}

////////////////////////////////////////////////////////////////////////////////

void DynamicLineResource::set_vertex_rendering_mode(scm::gl::primitive_topology const &render_mode)
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
    vertex_rendering_mode_ = render_mode;
}

////////////////////////////////////////////////////////////////////////////////

void DynamicLineResource::forward_queued_vertices(std::vector<scm::math::vec3f> const &queued_positions, std::vector<scm::math::vec4f> const &queued_colors,
                                                  std::vector<float> const &queued_thicknesses)
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
    std::dynamic_pointer_cast<DynamicGeometry>(dynamic_geometry_ptr_)->forward_queued_vertices(queued_positions, queued_colors, queued_thicknesses);
    compute_bounding_box();
    make_clean_flags_dirty();
}

////////////////////////////////////////////////////////////////////////////////

math::vec3 DynamicLineResource::get_vertex(unsigned int i) const { return math::vec3(dynamic_geometry_ptr_->positions[i]); }

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
