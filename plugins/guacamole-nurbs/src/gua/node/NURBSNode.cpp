#/******************************************************************************
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
#include "gua/node/NURBSNode.hpp"

#include <algorithm>

#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/renderer/NURBSLoader.hpp>
#include <gua/renderer/NURBSResource.hpp>
#include <gua/renderer/Material.hpp>

// guacamole headers

namespace gua
{
namespace node
{
////////////////////////////////////////////////////////////////////////////////
NURBSNode::NURBSNode(std::string const& name, std::string const& geometry_description, std::shared_ptr<Material> const& material, math::mat4 const& transform)
    : GeometryNode(name, transform), geometry_description_(geometry_description), geometry_changed_(true), material_(material)
{
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<NURBSResource> const& NURBSNode::get_geometry() const { return geometry_; }

////////////////////////////////////////////////////////////////////////////////
std::string const& NURBSNode::get_geometry_description() const { return geometry_description_; }

////////////////////////////////////////////////////////////////////////////////
void NURBSNode::set_geometry_description(std::string const& v)
{
    geometry_description_ = v;
    geometry_changed_ = self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Material> const& NURBSNode::get_material() const { return material_; }

////////////////////////////////////////////////////////////////////////////////
void NURBSNode::set_material(std::shared_ptr<Material> const& material)
{
    material_ = material;
    material_changed_ = self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
float NURBSNode::max_pre_tesselation() const { return max_pre_tesselation_; }

////////////////////////////////////////////////////////////////////////////////
void NURBSNode::max_pre_tesselation(float t) { max_pre_tesselation_ = std::max(1.0f, std::min(t, 64.0f)); }

////////////////////////////////////////////////////////////////////////////////
float NURBSNode::max_tesselation_error() const { return max_tesselation_error_; }

////////////////////////////////////////////////////////////////////////////////
void NURBSNode::max_tesselation_error(float t) { max_tesselation_error_ = std::max(1.0f, std::min(t, 64.0f)); }

////////////////////////////////////////////////////////////////////////////////
void NURBSNode::wireframe(bool enable)
{
    wireframe_ = enable;
    geometry_->wireframe(enable);
    geometry_changed_ = true;
}

////////////////////////////////////////////////////////////////////////////////
bool NURBSNode::wireframe() const { return wireframe_; }

////////////////////////////////////////////////////////////////////////////////
void NURBSNode::trimming(bool enable) { trimming_ = enable; }

////////////////////////////////////////////////////////////////////////////////
bool NURBSNode::trimming() const { return trimming_; }

////////////////////////////////////////////////////////////////////////////////
void NURBSNode::ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits)
{
    // very simple bounding box picking

    // first of all, check bbox
    auto box_hits(::gua::intersect(ray, bounding_box_));

    // ray did not intersect bbox -- therefore it wont intersect
    if(box_hits.first == Ray::END && box_hits.second == Ray::END)
    {
        return;
    }

    // return if only first object shall be returned and the current first hit
    // is in front of the bbox entry point and the ray does not start inside
    // the bbox
    if(options & PickResult::PICK_ONLY_FIRST_OBJECT && hits.size() > 0 && hits.begin()->distance < box_hits.first && box_hits.first != Ray::END)
    {
        return;
    }

    PickResult closest_hit = PickResult(0.f, this, ray.origin_, math::vec3(0.f, 0.f, 0.f), math::vec3(0.f, 1.f, 0.f), math::vec3(0.f, 1.f, 0.f), math::vec2(0.f, 0.f));

    const auto model_transform = this->get_cached_world_transform();
    const auto world_origin = ray.origin_;
    const auto world_direction = scm::math::normalize(ray.direction_);

    closest_hit.distance = box_hits.first;
    auto intersection_world = ray.origin_ + box_hits.first * ray.direction_;
    closest_hit.world_position = intersection_world;
    auto object_position = scm::math::inverse(model_transform) * gua::math::vec4(intersection_world.x, intersection_world.y, intersection_world.z, 1.0);
    closest_hit.position = math::vec3(object_position.x, object_position.y, object_position.z);

    math::vec4 normal_object_space;
    const double pick_epsilon = 0.00001;

    // x-direction
    if(std::fabs(geometry_->get_bounding_box().min.x - object_position.x) < pick_epsilon)
    {
        normal_object_space = math::vec4(-1.0, 0.0, 0.0, 0.0);
    }
    if(std::fabs(geometry_->get_bounding_box().max.x - object_position.x) < pick_epsilon)
    {
        normal_object_space = math::vec4(1.0, 0.0, 0.0, 0.0);
    }

    // y-direction
    if(std::fabs(geometry_->get_bounding_box().min.y - object_position.y) < pick_epsilon)
    {
        normal_object_space = math::vec4(1.0, -1.0, 0.0, 0.0);
    }
    if(std::fabs(geometry_->get_bounding_box().max.y - object_position.y) < pick_epsilon)
    {
        normal_object_space = math::vec4(0.0, -1.0, 0.0, 0.0);
    }

    // z-direction
    if(std::fabs(geometry_->get_bounding_box().min.z - object_position.z) < pick_epsilon)
    {
        normal_object_space = math::vec4(0.0, 0.0, -1.0, 0.0);
    }
    if(std::fabs(geometry_->get_bounding_box().max.z - object_position.x) < pick_epsilon)
    {
        normal_object_space = math::vec4(0.0, 0.0, 1.0, 0.0);
    }

    auto normal_world_space = model_transform * normal_object_space;
    closest_hit.normal = scm::math::normalize(math::vec3(normal_world_space.x, normal_world_space.y, normal_world_space.z));

    hits.insert(closest_hit);

    for(auto child : get_children())
    {
        child->ray_test_impl(ray, options, mask, hits);
    }
}

////////////////////////////////////////////////////////////////////////////////
void NURBSNode::update_bounding_box() const
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
void NURBSNode::update_cache()
{
    // The code below auto-loads a geometry if it's not already supported by
    // the GeometryDatabase. Name is generated by GeometryDescription

    if(geometry_changed_)
    {
        if(geometry_description_ != "")
        {
            if(!GeometryDatabase::instance()->contains(geometry_description_))
            {
                GeometryDescription desc(geometry_description_);
                try
                {
                    gua::NURBSLoader loader;
                    loader.load_geometry(get_name(), desc.filepath(), get_material(), desc.flags());
                }
                catch(std::exception& e)
                {
                    Logger::LOG_WARNING << "NURBSNode::update_cache(): Loading failed from " << desc.filepath() << " : " << e.what() << std::endl;
                }
            }

            geometry_ = std::dynamic_pointer_cast<NURBSResource>(GeometryDatabase::instance()->lookup(geometry_description_));

            if(!geometry_)
            {
                Logger::LOG_WARNING << "Failed to get TriMeshRessource for " << geometry_description_ << ": The data base entry is of wrong type!" << std::endl;
            }
        }

        geometry_changed_ = false;
    }

    GeometryNode::update_cache();
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void NURBSNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Node> NURBSNode::copy() const
{
    std::shared_ptr<NURBSNode> result = std::make_shared<NURBSNode>(*this);

    result->update_cache();

    result->shadow_mode_ = shadow_mode_;
    result->max_tesselation_error(this->max_tesselation_error());
    result->max_pre_tesselation(this->max_pre_tesselation());

    return result;
}

} // namespace node
} // namespace gua
