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
#include <gua/renderer/GeometryLoader.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/scenegraph/RayNode.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua {
  
  ////////////////////////////////////////////////////////////////////////////////
  GeometryNode::GeometryNode(std::string const& name,
                             std::string const& filename,
                             std::string const& material,
                             math::mat4 const& transform)
      : Node(name, transform), 
        filename_(filename),
        material_(material),
        filename_changed_(false), material_changed_(false) 
  {}

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
  void GeometryNode::update_cache() {

    // The code below auto-loads a geometry if it's not already supported by
    // the GeometryDatabase. It expects a geometry name like
    //
    // "type='file'&file='data/objects/monkey.obj'&id=0&flags=0"

    if (filename_changed_) {
      if (filename_ != "") {
        if (!GeometryDatabase::instance()->is_supported(filename_)) {
          auto params(string_utils::split(filename_, '&'));
          if (params.size() == 4) {
            if (params[0] == "type=file") {
              auto tmp_filename(string_utils::split(params[1], '='));
              auto tmp_flags(string_utils::split(params[3], '='));
              if (tmp_filename.size() == 2 && tmp_flags.size() == 2) {
                std::string filename(tmp_filename[1]);
                std::string flags_string(tmp_flags[1]);
                unsigned flags(0);
                std::stringstream sstr(flags_string);
                sstr >> flags;

                GeometryLoader loader;
                loader.load_geometry(filename, flags);

              }
              else {
                Logger::LOG_WARNING << "Failed to auto-load geometry " << filename_ << ": Failed to extract filename and/or loading flags!" << std::endl;
              }
            }
            else {
              Logger::LOG_WARNING << "Failed to auto-load geometry " << filename_ << ": Type is not supported!" << std::endl;
            }
          }
          else {
            Logger::LOG_WARNING << "Failed to auto-load geometry " << filename_ << ": The name does not contain a type, file, id and flag parameter!" << std::endl;
          }
        }
      }

      filename_changed_ = false;
    }

    // The code below auto-loads a material if it's not already supported by
    // the MaterialDatabase. It expects a material name like
    //
    // data/materials/Stones.gmd

    if (material_changed_) {
      if (material_ != "") {
        if (!MaterialDatabase::instance()->is_supported(material_)) {
          auto mat = std::make_shared<Material>(material_, MaterialDescription(material_));
          MaterialDatabase::instance()->add(material_, mat);
        }
      }

      material_changed_ = false;
    }

    Node::update_cache();
  }

}
