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
#include <gua/scenegraph/Video3DNode.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/renderer/Video3DLoader.hpp> 
#include <gua/renderer/Video3DUberShader.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/scenegraph/RayNode.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua {

  /////////////////////////////////////////////////////////////////////////////
  Video3DNode::Video3DNode(std::string const& name,
                           std::string const& file,
                           std::string const& material,
                           math::mat4 const& transform)
  : GeometryNode(name, file, material, transform)
  {
    // approximately local space
    bounding_box_ = math::BoundingBox<math::vec3>(math::vec3(-3.0, -0.1,-3.0),
                                                  math::vec3( 3.0, 2.5, 3.0)); 
  }

  /////////////////////////////////////////////////////////////////////////////

  void Video3DNode::update_bounding_box() const {
    //Logger::LOG_WARNING << "not implemented: Video3DNode::update_bounding_box()" << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////////

  void Video3DNode::ray_test_impl(RayNode const& ray, PickResult::Options options,
                             Mask const& mask, std::set<PickResult>& hits) {
    Logger::LOG_WARNING << "not implemented: Video3DNode::ray_test_impl()" << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<Node> Video3DNode::copy() const {
    return std::make_shared<Video3DNode>(get_name(), filename_, material_, get_transform());
  }

  /////////////////////////////////////////////////////////////////////////////

  /* virtual */ void Video3DNode::update_cache() 
  {
    if (filename_changed_)
    {
      if (filename_ != "")
      {
        if (!GeometryDatabase::instance()->is_supported(filename_))
        {
          Video3DLoader loader;
          loader.create_geometry_from_file("dummy", filename_);
        }
        else {
          Logger::LOG_WARNING << "Failed to auto-load geometry " << filename_ << ": Type is not supported!" << std::endl;
        }
      }
      else {
        Logger::LOG_WARNING << "Failed to auto-load geometry " << filename_ << ": The name does not contain a type, file, id and flag parameter!" << std::endl;
      }

      filename_changed_ = false;
    }

    // The code below auto-loads a material if it's not already supported by
    // the MaterialDatabase. It expects a material name like
    //
    // data/materials/Stones.gmd

    if (material_changed_)
    {
      if (material_ != "")
      {
        if (!MaterialDatabase::instance()->is_supported(material_) &&
            material_ != Video3DUberShader::default_video_material_name() )
        {
          auto mat = std::make_shared<Material>(material_, MaterialDescription(material_));
          MaterialDatabase::instance()->add(material_, mat);
        }
      }

      material_changed_ = false;
    }

    Node::update_cache();
  }

}
