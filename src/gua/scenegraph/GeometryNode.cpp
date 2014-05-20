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
#include <gua/scenegraph/GeometryNode.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/scenegraph/RayNode.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua {
  
  ////////////////////////////////////////////////////////////////////////////////
  GeometryNode::GeometryNode(std::string const& name,
                             std::string const& filename,
                             std::string const& material,
                             math::mat4 const& transform,
                             ShadowMode shadow_mode)
      : Node(name, transform), 
        filename_(filename),
        material_(material),
        shadow_mode_(shadow_mode),
        filename_changed_(false), 
        material_changed_(false) 
  {
    cache_material();
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::string const& GeometryNode::get_filename() const {
    return filename_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void GeometryNode::set_filename(std::string const& v) {
    filename_ = v;
    filename_changed_ = self_dirty_ = true;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /* virtual */ void GeometryNode::accept(NodeVisitor& visitor) {
    visitor.visit(this);
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::string const& GeometryNode::get_material() const {
    return material_; 
  }

  ////////////////////////////////////////////////////////////////////////////////
  void GeometryNode::set_material(std::string const& v) {
    material_ = v; 
    material_changed_ = self_dirty_ = true; 
    cache_material();
  }

  ////////////////////////////////////////////////////////////////////////////////
  void GeometryNode::update_bounding_box() const {

    if (get_filename() != "") {
      auto geometry_bbox(GeometryDatabase::instance()->lookup(get_filename())->get_bounding_box());
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
  void GeometryNode::cache_material() const
  {
    if (!material_.empty())
    {
      if (!MaterialDatabase::instance()->is_supported(material_))
      {
        MaterialDatabase::instance()->load_material(material_);
      }
    }
  }
}
