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
#include <gua/spoints/SPointsNode.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/spoints/SPointsLoader.hpp>
#include <gua/spoints/SPointsRenderer.hpp>
#include <gua/spoints/SPointsResource.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua
{
namespace node
{
/////////////////////////////////////////////////////////////////////////////
SPointsNode::SPointsNode(std::string const& name, std::string const& spoints_description, std::shared_ptr<Material> const& material, math::mat4 const& transform)
    : GeometryNode(name, transform), screen_space_point_size_(1.0f), spoints_(nullptr), spoints_description_(spoints_description), spoints_changed_(true), material_(material), material_changed_(true),
      is_server_resource_(false)
{
}

/////////////////////////////////////////////////////////////////////////////

void SPointsNode::ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits)
{
    if(!spoints_->is_pickable())
    {
        return;
    }

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
}

/////////////////////////////////////////////////////////////////////////////

/* virtual */ void SPointsNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

/////////////////////////////////////////////////////////////////////////////
std::string const& SPointsNode::get_spoints_description() const { return spoints_description_; }

/////////////////////////////////////////////////////////////////////////////
void SPointsNode::set_spoints_description(std::string const& v)
{
    spoints_description_ = v;
    spoints_changed_ = self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void SPointsNode::force_reload() { spoints_changed_ = true; }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Material> const& SPointsNode::get_material() const { return material_; }

////////////////////////////////////////////////////////////////////////////////
void SPointsNode::set_material(std::shared_ptr<Material> const& material)
{
    material_ = material;
    spoints_changed_ = self_dirty_ = true;
}

/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> SPointsNode::copy() const { return std::make_shared<SPointsNode>(*this); }

/////////////////////////////////////////////////////////////////////////////

/* virtual */ void SPointsNode::update_cache()
{
    if(spoints_changed_)
    {
        if(spoints_description_ != "")
        {
            if(!GeometryDatabase::instance()->contains(spoints_description_))
            {
                GeometryDescription desc(spoints_description_);
                try
                {
                    gua::SPointsLoader loader;
                    loader.create_geometry_from_file("tmp", desc.filepath(), nullptr, desc.flags());
                    Logger::LOG_WARNING << "SPointsNode::update_cache(): Loading " << desc.filepath() << std::endl;
                }
                catch(std::exception& e)
                {
                    Logger::LOG_WARNING << "SPointsNode::update_cache(): Loading failed from " << desc.filepath() << " : " << e.what() << std::endl;
                }
            }

            spoints_ = std::dynamic_pointer_cast<SPointsResource>(GeometryDatabase::instance()->lookup(spoints_description_));

            if(!spoints_)
            {
                Logger::LOG_WARNING << "Failed to get SPoints3DResource for " << spoints_description_ << ": The data base entry is of wrong type!" << std::endl;
            }
        }

        spoints_changed_ = false;
    }

    Node::update_cache();
}

////////////////////////////////////////////////////////////////////////////////

void SPointsNode::set_is_server_resource(bool is_server_resource) { is_server_resource_ = is_server_resource; }

////////////////////////////////////////////////////////////////////////////////

bool SPointsNode::get_is_server_resource() const { return is_server_resource_; }

} // namespace node
} // namespace gua
