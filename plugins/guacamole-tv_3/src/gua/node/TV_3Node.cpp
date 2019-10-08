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
#include "gua/node/TV_3Node.hpp"

#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/renderer/TV_3Loader.hpp>
#include <gua/renderer/TV_3Resource.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

// guacamole headers

namespace gua
{
namespace node
{
////////////////////////////////////////////////////////////////////////////////
TV_3Node::TV_3Node(std::string const& name, std::string const& geometry_description, std::string const& geometry_file_path, std::shared_ptr<Material> const& material, math::mat4 const& transform)
    : GeometryNode(name, transform), geometry_(nullptr), geometry_changed_(true), geometry_description_(geometry_description), geometry_file_path_(geometry_file_path), material_(material)
{
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<TV_3Resource> const& TV_3Node::get_geometry() const { return geometry_; }

////////////////////////////////////////////////////////////////////////////////
std::string const& TV_3Node::get_geometry_file_path() const { return geometry_file_path_; }

////////////////////////////////////////////////////////////////////////////////
std::string const& TV_3Node::get_geometry_description() const { return geometry_description_; }

////////////////////////////////////////////////////////////////////////////////
void TV_3Node::set_geometry_description(std::string const& v)
{
    geometry_description_ = v;
    geometry_changed_ = self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Material> const& TV_3Node::get_material() const { return material_; }

////////////////////////////////////////////////////////////////////////////////
void TV_3Node::set_material(std::shared_ptr<Material> const& material)
{
    material_ = material;
    // material_changed_ = self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void TV_3Node::ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits)
{
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

    // bbox is intersected, but check geometry only if mask tells us to check
    if(get_geometry_description() != "" && mask.check(get_tags()))
    {
        auto geometry(GeometryDatabase::instance()->lookup(get_geometry_description()));

        if(geometry)
        {
            Ray world_ray(ray);
            geometry->ray_test(world_ray, options, this, hits);
        }
    }

    for(auto child : get_children())
    {
        child->ray_test_impl(ray, options, mask, hits);
    }
}

////////////////////////////////////////////////////////////////////////////////
void TV_3Node::update_bounding_box() const
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
void TV_3Node::update_cache()
{
    if(geometry_changed_)
    {
        if(geometry_description_ != "")
        {
            if(!GeometryDatabase::instance()->contains(geometry_description_))
            {
                GeometryDescription desc(geometry_description_);
                try
                {
                    gua::TV_3Loader loader;
                    loader.load_geometry(desc.filepath(), desc.flags());
                }
                catch(std::exception& e)
                {
                    Logger::LOG_WARNING << "TV_3Node::update_cache(): Loading failed from " << desc.filepath() << " : " << e.what() << std::endl;
                }
            }
            geometry_ = std::dynamic_pointer_cast<TV_3Resource>(GeometryDatabase::instance()->lookup(geometry_description_));

            if(!geometry_)
            {
                Logger::LOG_WARNING << "Failed to get TV_3Resource for " << geometry_description_ << ": The data base entry is of wrong type!" << std::endl;
            }
        }

        geometry_changed_ = false;
    }

    // modified version of Node::update_cache -> add local transformation
    if(self_dirty_)
    {
        math::mat4 old_world_trans(world_transform_);

        if(is_root())
        {
            world_transform_ = transform_ * geometry_->local_transform();
        }
        else
        {
            world_transform_ = get_parent()->get_world_transform() * transform_ * geometry_->local_transform();
        }

        update_bounding_box();

        if(world_transform_ != old_world_trans)
        {
            on_world_transform_changed.emit(world_transform_);
        }

        self_dirty_ = false;
    }

    if(child_dirty_)
    {
        for(auto const& child : get_children())
        {
            child->update_cache();
        }

        update_bounding_box();

        child_dirty_ = false;
    }
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void TV_3Node::accept(NodeVisitor& visitor) { visitor.visit(this); }

/*
////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<TV_3Node> TV_3Node::deep_copy_TV3() const {
  std::shared_ptr<TV_3Node> copied_node = copy_TV3();
  copied_node->tags_ = tags_;
  copied_node->draw_bounding_box_ = draw_bounding_box_;
  copied_node->scenegraph_ = scenegraph_;
  copied_node->bounding_box_ = bounding_box_;
  copied_node->user_data_ = user_data_;
  copied_node->world_transform_ = world_transform_;
  copied_node->uuid_ = uuid_;

  for (int i(0); i<children_.size(); ++i) {
    copied_node->children_[i] = children_[i]->deep_copy_TV3();
    copied_node->children_[i]->parent_ = copied_node.get();
  }


  return copied_node;
}
*/
////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Node> TV_3Node::copy() const
{
    std::shared_ptr<TV_3Node> result = std::make_shared<TV_3Node>(*this);

    result->update_cache();

    result->shadow_mode_ = shadow_mode_;
    result->render_mode_ = render_mode_;
    result->iso_value_ = iso_value_;
    result->spatial_filter_mode_ = spatial_filter_mode_;
    result->temporal_filter_mode_ = temporal_filter_mode_;

    return result;
}

} // namespace node
} // namespace gua
