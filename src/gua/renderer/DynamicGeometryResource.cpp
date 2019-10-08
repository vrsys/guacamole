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
#include <gua/renderer/DynamicGeometryResource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/node/DynamicGeometryNode.hpp>
#include <gua/utils/Logger.hpp>

#include <scm/gl_core/constants.h>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

DynamicGeometryResource::DynamicGeometryResource(scm::gl::primitive_topology vertex_rendering_mode)
    : kd_tree_(), dynamic_geometry_ptr_(), vertex_rendering_mode_(vertex_rendering_mode), clean_flags_per_context_()
{
    // : kd_tree_(), dynamic_geometry_(), vertex_rendering_mode_(scm::gl::PRIMITIVE_LINE_LIST), clean_flags_per_context_() {
    compute_bounding_box();
}

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

void DynamicGeometryResource::compute_bounding_box()
{
    // if (dynamic_geometry_.num_occupied_vertex_slots > 0) {
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

DynamicGeometryResource::DynamicGeometryResource(std::shared_ptr<DynamicGeometry> dynamic_geometry_ptr, bool build_kd_tree, scm::gl::primitive_topology vertex_rendering_mode)
    : kd_tree_(), dynamic_geometry_ptr_(dynamic_geometry_ptr), vertex_rendering_mode_(vertex_rendering_mode), clean_flags_per_context_()
{
    compute_bounding_box();

    if(build_kd_tree)
    {
        // kd_tree_.generate(dynamic_geometry);
    }
}

////////////////////////////////////////////////////////////////////////////////

void DynamicGeometryResource::upload_to(RenderContext &ctx) const
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);

    /*
      if (dynamic_geometry_.vertex_reservoir_size == 0) {
        Logger::LOG_WARNING << "Unable to load DynamicGeometry! Has no vertex data." << std::endl;
        return;
      }
    */
    auto dynamic_geometry_iterator = ctx.dynamic_geometries.find(uuid());
    // auto dynamic_geometry_iterator = std::dynamic_pointer_cast<DynamicGeometry>(ctx.dynamic_geometries.find(uuid()));
    // auto dynamic_geometry_iterator = std::dynamic_pointer_cast<DynamicGeometry>(ctx.dynamic_geometries.find(uuid());

    bool update_cached_dynamic_geometry{false};

    if((ctx.dynamic_geometries.end() == dynamic_geometry_iterator))
    {
        ctx.dynamic_geometries[uuid()] = RenderContext::DynamicGeometry();
    }

    RenderContext::DynamicGeometry *dynamic_geometry_to_update_ptr = &ctx.dynamic_geometries[uuid()];

    if(update_cached_dynamic_geometry)
    {
        dynamic_geometry_to_update_ptr = &(dynamic_geometry_iterator->second);
    }

    // = mode_;
    // dynamic_geometry_to_update_ptr->vertex_topology = scm::gl::PRIMITIVE_LINE_LIST;
    // dynamic_geometry_to_update_ptr->vertex_topology = scm::gl::PRIMITIVE_TRIANGLE_LIST;
    dynamic_geometry_to_update_ptr->vertex_topology = vertex_rendering_mode_;
    dynamic_geometry_to_update_ptr->vertex_reservoir_size = dynamic_geometry_ptr_->vertex_reservoir_size;
    dynamic_geometry_to_update_ptr->num_occupied_vertex_slots = dynamic_geometry_ptr_->num_occupied_vertex_slots;

    if(dynamic_geometry_to_update_ptr->current_buffer_size_in_vertices < dynamic_geometry_ptr_->vertex_reservoir_size)
    {
        dynamic_geometry_to_update_ptr->vertices =
            ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_DYNAMIC_DRAW, (dynamic_geometry_ptr_->vertex_reservoir_size) * sizeof(DynamicGeometry::Vertex), 0);

        dynamic_geometry_to_update_ptr->current_buffer_size_in_vertices = dynamic_geometry_ptr_->vertex_reservoir_size;
    }
    else
    {
        update_cached_dynamic_geometry = true;
    }

    if(dynamic_geometry_ptr_->vertex_reservoir_size != 0)
    {
        DynamicGeometry::Vertex *data(static_cast<DynamicGeometry::Vertex *>(ctx.render_context->map_buffer(dynamic_geometry_to_update_ptr->vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

        dynamic_geometry_ptr_->copy_to_buffer(data);
        ctx.render_context->unmap_buffer(dynamic_geometry_to_update_ptr->vertices);

        dynamic_geometry_to_update_ptr->vertex_array = ctx.render_device->create_vertex_array(dynamic_geometry_ptr_->get_vertex_format(), {dynamic_geometry_to_update_ptr->vertices});
    }

    ctx.render_context->apply();
}

////////////////////////////////////////////////////////////////////////////////

void DynamicGeometryResource::ray_test(Ray const &ray, int options, node::Node *owner, std::set<PickResult> &hits)
{
    // kd_tree_.ray_test(ray, dynamic_geometry_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////
/*
void DynamicGeometryResource::resolve_vertex_updates(RenderContext& ctx) {

  //TODO: PROTECT BY LINE STRIP UPDATE MUTEX

  for(auto& update_job_ptr : dynamic_geometry_update_queue_) {
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

void DynamicGeometryResource::make_clean_flags_dirty()
{
    for(auto &known_clean_flag : clean_flags_per_context_)
    {
        known_clean_flag.second = false;
    }
}

////////////////////////////////////////////////////////////////////////////////

void DynamicGeometryResource::compute_consistent_normals() const
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
    dynamic_geometry_ptr_->compute_consistent_normals();
}

////////////////////////////////////////////////////////////////////////////////
// pure virutal now
// void DynamicGeometryResource::compile_buffer_string(std::string &buffer_string)
// {
//     std::cout << "DGR compile_buffer_string" << std::endl;
//     std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
//     dynamic_geometry_ptr_->compile_buffer_string(buffer_string);
// };

// ////////////////////////////////////////////////////////////////////////////////

// void DynamicGeometryResource::uncompile_buffer_string(std::string const &buffer_string)
// {
//     std::cout << "DGR uncompile_buffer_string" << std::endl;
//     std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
//     dynamic_geometry_ptr_->uncompile_buffer_string(buffer_string);
//     make_clean_flags_dirty();
// };

////////////////////////////////////////////////////////////////////////////////

void DynamicGeometryResource::push_vertex(DynamicGeometry::Vertex const &in_vertex)
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);

    if(dynamic_geometry_ptr_->push_vertex(in_vertex))
    {
        if(dynamic_geometry_ptr_->num_occupied_vertex_slots > 0)
        {
            bounding_box_.expandBy(math::vec3{dynamic_geometry_ptr_->positions[dynamic_geometry_ptr_->num_occupied_vertex_slots - 1]});
        }
        make_clean_flags_dirty();
    }
}

////////////////////////////////////////////////////////////////////////////////

void DynamicGeometryResource::pop_front_vertex()
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
    if(dynamic_geometry_ptr_->pop_front_vertex())
    {
        compute_bounding_box();
        make_clean_flags_dirty();
    }
}

////////////////////////////////////////////////////////////////////////////////

void DynamicGeometryResource::pop_back_vertex()
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
    if(dynamic_geometry_ptr_->pop_back_vertex())
    {
        compute_bounding_box();
        make_clean_flags_dirty();
    }
}

////////////////////////////////////////////////////////////////////////////////

void DynamicGeometryResource::clear_vertices()
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
    if(dynamic_geometry_ptr_->clear_vertices())
    {
        compute_bounding_box();
        make_clean_flags_dirty();
    }
}

void DynamicGeometryResource::set_vertex_rendering_mode(scm::gl::primitive_topology const &render_mode)
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
    vertex_rendering_mode_ = render_mode;
}

////////////////////////////////////////////////////////////////////////////////

void DynamicGeometryResource::forward_queued_vertices(std::vector<scm::math::vec3f> const &queued_positions, std::vector<scm::math::vec4f> const &queued_colors,
                                                      std::vector<float> const &queued_thicknesses //,
                                                      // std::vector<scm::math::vec3f> const& queued_normals
)
{
    std::lock_guard<std::mutex> lock(dynamic_geometry_update_mutex_);
    dynamic_geometry_ptr_->forward_queued_vertices(queued_positions, queued_colors,
                                                   queued_thicknesses //,
                                                   // queued_normals
    );
    compute_bounding_box();
    make_clean_flags_dirty();
}

////////////////////////////////////////////////////////////////////////////////

math::vec3 DynamicGeometryResource::get_vertex(unsigned int i) const
{
    return math::vec3(dynamic_geometry_ptr_->positions[i].x, dynamic_geometry_ptr_->positions[i].y, dynamic_geometry_ptr_->positions[i].z);
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
