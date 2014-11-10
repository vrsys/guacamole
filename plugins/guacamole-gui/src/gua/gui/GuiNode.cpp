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
#include <gua/gui/GuiNode.hpp>

#include <gua/gui/GuiResource.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

// guacamole headers

namespace gua {
namespace gui {

  ////////////////////////////////////////////////////////////////////////////////
  GuiNode::GuiNode(std::string const& name,
                   std::string const& resource_name = "",
                   math::mat4 const& transform)
    : GeometryNode(name, transform),
      resource_(nullptr)
      resource_name_(resource_name),
      resource_name_changed_(true)
  {}

  ////////////////////////////////////////////////////////////////////////////////

  void GuiNode::ray_test_impl(Ray const& ray, PickResult::Options options,
    Mask const& mask, std::set<PickResult>& hits) {

  }


  ////////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<GeometryResource> const& GuiNode::get_resource() const {
    return resource_;
  }

  ////////////////////////////////////////////////////////////////////////////////

  /* virtual */ void GuiNode::accept(NodeVisitor& visitor) {
    visitor.visit(this);
  }

  ////////////////////////////////////////////////////////////////////////////////

  void GuiNode::update_bounding_box() const {

  }

  ////////////////////////////////////////////////////////////////////////////////

  void TriMeshNode::update_cache() {

    if (resource_name_changed_) {
      if (resource_name_ != "") {
        if (!GeometryDatabase::instance()->is_supported(resource_name_)) {

          resource_ = std::make_shared<GuiResource>(resource_name_);
          GeometryDatabase::instance()->add(resource_name_, resource_);

        } else {

          resource_ = GeometryDatabase::instance()->lookup(resource_name_);
        }

      }

      resource_name_changed_ = false;
    }

    GeometryNode::update_cache();
  }

  ////////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<Node> GuiNode::copy() const {
    auto result(std::make_shared<GuiNode>(get_name(), get_transform()));
    result->shadow_mode_ = shadow_mode_;
    result->resource_ = resource_;
    return result;
  }
}
}
