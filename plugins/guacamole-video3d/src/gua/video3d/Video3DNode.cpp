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
#include <gua/video3d/Video3DNode.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/video3d/Video3DLoader.hpp>
#include <gua/video3d/Video3DRenderer.hpp>
#include <gua/video3d/Video3DResource.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua
{
namespace node
{
/////////////////////////////////////////////////////////////////////////////
Video3DNode::Video3DNode(std::string const& name, std::string const& video_description, std::shared_ptr<Material> const& material, math::mat4 const& transform)
    : GeometryNode(name, transform), video_(nullptr), video_description_(video_description), video_changed_(true), material_(material), material_changed_(true)
{
}

/////////////////////////////////////////////////////////////////////////////

void Video3DNode::ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits)
{
    if(!video_->is_pickable())
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

/* virtual */ void Video3DNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

/////////////////////////////////////////////////////////////////////////////
std::string const& Video3DNode::get_video_description() const { return video_description_; }

/////////////////////////////////////////////////////////////////////////////
void Video3DNode::set_video_description(std::string const& v)
{
    video_description_ = v;
    video_changed_ = self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void Video3DNode::force_reload() { video_changed_ = true; }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Material> const& Video3DNode::get_material() const { return material_; }

////////////////////////////////////////////////////////////////////////////////
void Video3DNode::set_material(std::shared_ptr<Material> const& material)
{
    material_ = material;
    video_changed_ = self_dirty_ = true;
}

/////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> Video3DNode::copy() const { return std::make_shared<Video3DNode>(*this); }

/////////////////////////////////////////////////////////////////////////////

/* virtual */ void Video3DNode::update_cache()
{
    if(video_changed_)
    {
        if(video_description_ != "")
        {
            if(!GeometryDatabase::instance()->contains(video_description_))
            {
                GeometryDescription desc(video_description_);
                try
                {
                    gua::Video3DLoader loader;
                    loader.create_geometry_from_file("tmp", desc.filepath(), nullptr, desc.flags());
                    Logger::LOG_WARNING << "Video3DNode::update_cache(): Loading " << desc.filepath() << std::endl;
                }
                catch(std::exception& e)
                {
                    Logger::LOG_WARNING << "Video3DNode::update_cache(): Loading failed from " << desc.filepath() << " : " << e.what() << std::endl;
                }
            }

            video_ = std::dynamic_pointer_cast<Video3DResource>(GeometryDatabase::instance()->lookup(video_description_));

            if(!video_)
            {
                Logger::LOG_WARNING << "Failed to get VideoResource for " << video_description_ << ": The data base entry is of wrong type!" << std::endl;
            }
        }

        video_changed_ = false;
    }

    Node::update_cache();
}
} // namespace node
} // namespace gua
