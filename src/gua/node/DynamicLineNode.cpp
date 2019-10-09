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
#include "gua/node/DynamicLineNode.hpp"

// guacamole headers
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/renderer/DynamicGeometryLoader.hpp>
#include <gua/renderer/DynamicLineLoader.hpp>
#include <gua/renderer/DynamicGeometryResource.hpp>
#include <gua/renderer/DynamicLineResource.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua
{
namespace node
{
////////////////////////////////////////////////////////////////////////////////
DynamicLineNode::DynamicLineNode(std::string const& name, std::string const& geometry_description, std::shared_ptr<Material> const& material, math::mat4 const& transform)
    : DynamicGeometryNode(name, geometry_description, material, transform)
{
}

////////////////////////////////////////////////////////////////////////////////

void DynamicLineNode::ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits)
{
    // first of all, check bbox
    auto box_hits(::gua::intersect(ray, bounding_box_));

    // ray did not intersect bbox -- therefore it wont intersect
    if(box_hits.first == Ray::END && box_hits.second == Ray::END)
    {
        return;
    }

    // return if only first object shall be returned and the current first hit
    // is in front of the bbox entry point
    if(options & PickResult::PICK_ONLY_FIRST_OBJECT && hits.size() > 0 && hits.begin()->distance < box_hits.first)
    {
        return;
    }

    // bbox is intersected, but check geometry only if mask tells us to check
    if(get_geometry_description() != "" && mask.check(get_tags()))
    {
        auto geometry(GeometryDatabase::instance()->lookup(get_geometry_description()));

        if(geometry)
        {
            bool check_kd_tree(true);

            math::mat4 world_transform(get_world_transform());

            // check for bounding box intersection of contained geometry if node
            // has children (in this case, the bbox might be larger
            // than the actual geometry)
            if(has_children())
            {
                auto geometry_bbox(geometry->get_bounding_box());

#if 0
          auto inner_bbox = gua::math::transform(geometry_bbox, world_transform);
#else
                math::BoundingBox<math::vec3> inner_bbox;
                inner_bbox.expandBy(world_transform * geometry_bbox.min);
                inner_bbox.expandBy(world_transform * geometry_bbox.max);
                inner_bbox.expandBy(world_transform * math::vec3(geometry_bbox.min.x, geometry_bbox.min.y, geometry_bbox.max.z));
                inner_bbox.expandBy(world_transform * math::vec3(geometry_bbox.min.x, geometry_bbox.max.y, geometry_bbox.min.z));
                inner_bbox.expandBy(world_transform * math::vec3(geometry_bbox.min.x, geometry_bbox.max.y, geometry_bbox.max.z));
                inner_bbox.expandBy(world_transform * math::vec3(geometry_bbox.max.x, geometry_bbox.min.y, geometry_bbox.max.z));
                inner_bbox.expandBy(world_transform * math::vec3(geometry_bbox.max.x, geometry_bbox.max.y, geometry_bbox.min.z));
                inner_bbox.expandBy(world_transform * math::vec3(geometry_bbox.max.x, geometry_bbox.min.y, geometry_bbox.min.z));
#endif

                auto inner_hits(::gua::intersect(ray, inner_bbox));
                if(inner_hits.first == RayNode::END && inner_hits.second == RayNode::END)
                    check_kd_tree = false;
            }

            if(check_kd_tree)
            {
                Ray world_ray(ray);

                math::mat4 ori_transform(scm::math::inverse(world_transform));

                math::vec4 ori(world_ray.origin_[0], world_ray.origin_[1], world_ray.origin_[2], 1.0);
                math::vec4 dir(world_ray.direction_[0], world_ray.direction_[1], world_ray.direction_[2], 0.0);

                ori = ori_transform * ori;
                dir = ori_transform * dir;

                Ray object_ray(ori, dir, world_ray.t_max_);
                geometry->ray_test(object_ray, options, this, hits);

                float const inf(std::numeric_limits<float>::max());

                if(options & PickResult::GET_WORLD_POSITIONS)
                {
                    for(auto& hit : hits)
                    {
                        if(hit.world_position == math::vec3(inf, inf, inf))
                        {
                            auto transformed(world_transform * math::vec4(hit.position.x, hit.position.y, hit.position.z, 1.0));
                            hit.world_position = scm::math::vec3(transformed.x, transformed.y, transformed.z);
                        }
                    }
                }

                if(options & PickResult::GET_WORLD_NORMALS)
                {
                    math::mat4 normal_matrix(scm::math::inverse(scm::math::transpose(world_transform)));
                    for(auto& hit : hits)
                    {
                        if(hit.world_normal == math::vec3(inf, inf, inf))
                        {
                            auto transformed(normal_matrix * math::vec4(hit.normal.x, hit.normal.y, hit.normal.z, 0.0));
                            hit.world_normal = scm::math::normalize(scm::math::vec3(transformed.x, transformed.y, transformed.z));
                        }
                    }
                }
            }
        }
    }

    for(auto child : get_children())
    {
        // test for intersection with each child
        child->ray_test_impl(ray, options, mask, hits);
    }
}

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<DynamicGeometryResource> const& DynamicLineNode::get_geometry() const { return geometry_; }

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void DynamicLineNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

////////////////////////////////////////////////////////////////////////////////
void DynamicLineNode::update_bounding_box() const
{
    if(geometry_)
    {
        auto geometry_bbox(geometry_->get_bounding_box());
        bounding_box_ = transform(geometry_bbox, world_transform_);

        for(auto child : get_children())
        {
            bounding_box_.expandBy(child->get_bounding_box());
        }
    }
    else
    {
        Node::update_bounding_box();
    }
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Node> DynamicLineNode::copy() const { return std::make_shared<DynamicLineNode>(*this); }

////////////////////////////////////////////////////////////////////////////////
void DynamicLineNode::push_vertex(DynamicGeometry::Vertex const& dynamic_vertex)
{
    push_vertex(dynamic_vertex.pos[0], dynamic_vertex.pos[1], dynamic_vertex.pos[2], dynamic_vertex.col[0], dynamic_vertex.col[1], dynamic_vertex.col[2], dynamic_vertex.col[3], dynamic_vertex.thick);
};

////////////////////////////////////////////////////////////////////////////////
void DynamicLineNode::enqueue_vertex(float x, float y, float z, float col_r, float col_g, float col_b, float col_a, float thickness)
{
    queued_positions_.push_back(scm::math::vec3f(x, y, z));
    queued_colors_.push_back(scm::math::vec4f(col_r, col_g, col_b, col_a));
    queued_thicknesses_.push_back(thickness);
};

////////////////////////////////////////////////////////////////////////////////
void DynamicLineNode::push_vertex(float x, float y, float z, float col_r, float col_g, float col_b, float col_a, float thickness)
{
    if(nullptr != geometry_)
    {
        DynamicGeometry::Vertex vertex_to_push(x, y, z, col_r, col_g, col_b, col_a, thickness);
        auto line_geometry = std::dynamic_pointer_cast<DynamicLineResource>(geometry_);
        line_geometry->push_vertex(vertex_to_push);
    }
};

void DynamicLineNode::update_vertex(int vertex_idx, float x, float y, float z, float col_r, float col_g, float col_b, float col_a, float thickness)
{
    if(nullptr != geometry_)
    {
        if(vertex_idx < geometry_->num_occupied_vertex_slots())
        {
            DynamicGeometry::Vertex vertex_to_update(x, y, z, col_r, col_g, col_b, col_a, thickness);
            auto line_geometry = std::dynamic_pointer_cast<DynamicLineResource>(geometry_);
            line_geometry->update_vertex(vertex_idx, vertex_to_update);
        }
        else
        {
            std::cout << "Vertex index doesnt exist." << std::endl;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
void DynamicLineNode::clear_vertices()
{
    if(nullptr != geometry_)
    {
        queued_positions_.clear();
        queued_colors_.clear();
        queued_thicknesses_.clear();
    }
}

////////////////////////////////////////////////////////////////////////////////
void DynamicLineNode::forward_queued_vertices()
{
    if(nullptr != geometry_)
    {
        auto line_geometry = std::dynamic_pointer_cast<DynamicLineResource>(geometry_);
        line_geometry->forward_queued_vertices(queued_positions_, queued_colors_, queued_thicknesses_);
        update_bounding_box();
    }
}

//    ////////////////////////////////////////////////////////////////////////////////
// void DynamicLineNode::set_geometry(std::shared_ptr<DynamicGeometryResource> res) {
//   std::cout<<"set tri GEOMETRY"<< res<< std::endl;
//   geometry_ = res;
// }

////////////////////////////////////////////////////////////////////////////////
void DynamicLineNode::compile_buffer_string(std::string& buffer_string)
{
    std::cout << " dtn compile buffer " << std::endl;
    if(nullptr != geometry_)
    {
        geometry_->compile_buffer_string(buffer_string);
    }
};

////////////////////////////////////////////////////////////////////////////////
void DynamicLineNode::uncompile_buffer_string(std::string const& buffer_string)
{
    if(nullptr != geometry_)
    {
        std::cout << "dtn uncompile buffer " << std::endl;
        geometry_->uncompile_buffer_string(buffer_string);
    }
};

void DynamicLineNode::update_geometry_cache(::gua::GeometryDescription const& desc)
{
    gua::DynamicLineLoader loader;
    loader.create_empty_geometry(get_name(), desc.filepath(), get_material(), desc.flags());
    std::cout << "update geometry_ cache " << std::endl;
}

} // namespace node
} // namespace gua
