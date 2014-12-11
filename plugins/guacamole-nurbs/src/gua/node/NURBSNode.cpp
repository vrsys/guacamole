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

namespace gua {
namespace node {

  ////////////////////////////////////////////////////////////////////////////////
  NURBSNode::NURBSNode(std::string const& name,
                       std::string const& geometry_description,
                       std::shared_ptr<Material> const& material,
                       math::mat4 const& transform)
    : GeometryNode(name, transform),
      geometry_description_(geometry_description),
      material_(material),
      max_tess_level_pre_pass_(1.0f),
      max_tess_level_final_pass_(4.0f)
  {}

  ////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<NURBSResource> const& NURBSNode::get_geometry() const
  {
    return geometry_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::string const& NURBSNode::get_geometry_description() const {
    return geometry_description_;
  }


  ////////////////////////////////////////////////////////////////////////////////
  void NURBSNode::set_geometry_description(std::string const& v) {
    geometry_description_ = v;
    geometry_changed_ = self_dirty_ = true;
  }


  ////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<Material> const& NURBSNode::get_material() const {
    return material_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void NURBSNode::set_material(std::shared_ptr<Material> const& material) {
    material_ = material;
    material_changed_ = self_dirty_ = true;
  }


  ////////////////////////////////////////////////////////////////////////////////
  float NURBSNode::max_pre_tesselation() const
  {
    return max_tess_level_pre_pass_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void NURBSNode::max_pre_tesselation(float t)
  {
    max_tess_level_pre_pass_ = std::max(1.0f, std::min(t, 64.0f));
  }

  ////////////////////////////////////////////////////////////////////////////////
  float NURBSNode::max_final_tesselation() const
  {
    return max_tess_level_final_pass_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void NURBSNode::max_final_tesselation(float t)
  {
    max_tess_level_final_pass_ = std::max(1.0f, std::min(t, 64.0f));
  }



  ////////////////////////////////////////////////////////////////////////////////
  void NURBSNode::ray_test_impl(Ray const& ray,
                                int options,
                                Mask const& mask,
                                std::set<PickResult>& hits) {
    Logger::LOG_WARNING << "NURBSNode::ray_test_impl() : Ray test not implemented yet for NURBS" << std::endl;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void NURBSNode::update_bounding_box() const
  {
    if (geometry_) {
      auto geometry_bbox(geometry_->get_bounding_box());
      bounding_box_ = transform(geometry_bbox, world_transform_);

      for (auto child : get_children()) {
        bounding_box_.expandBy(child->get_bounding_box());
      }
    }
    else {
      Node::update_bounding_box();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  void NURBSNode::update_cache() {
    // The code below auto-loads a geometry if it's not already supported by
    // the GeometryDatabase. Name is generated by GeometryDescription

    if (geometry_changed_)
    {
      if (geometry_description_ != "")
      {
        if (!GeometryDatabase::instance()->contains(geometry_description_))
        {
          GeometryDescription desc(geometry_description_);
          try {
            gua::NURBSLoader loader;
            loader.load_geometry(desc.filepath(), desc.flags());
          }
          catch (std::exception& e) {
            Logger::LOG_WARNING << "TriMeshNode::update_cache(): Loading failed from " << desc.filepath() << " : " << e.what() << std::endl;
          }
        }

        geometry_ = std::dynamic_pointer_cast<NURBSResource>(GeometryDatabase::instance()->lookup(geometry_description_));

        if (!geometry_) {
          Logger::LOG_WARNING << "Failed to get NURBSResource for " << geometry_description_ << ": The data base entry is of wrong type!" << std::endl;
        }
      }

      geometry_changed_ = false;
    }

    // The code below auto-loads a material if it's not already supported by
    // the MaterialShaderDatabase. It expects a material name like
    //
    // data/materials/Stones.gmd

    if (material_changed_)
    {
      if (!material_)
      {
        // if (!MaterialShaderDatabase::instance()->contains(material_))
        // {
        //   MaterialShaderDatabase::instance()->load_material(material_);
        // }
      }

      material_changed_ = false;
    }

    GeometryNode::update_cache();
  }


  ////////////////////////////////////////////////////////////////////////////////
  /* virtual */ void NURBSNode::accept(NodeVisitor& visitor) {
    visitor.visit(this);
  }


  ////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<Node> NURBSNode::copy() const 
  {
    std::shared_ptr<NURBSNode> result(new NURBSNode(get_name(), geometry_description_, material_, get_transform()));

    result->update_cache();

    result->shadow_mode_ = shadow_mode_;

    result->max_final_tesselation(this->max_final_tesselation());
    result->max_pre_tesselation(this->max_pre_tesselation());

    return result;
  }

}
}
