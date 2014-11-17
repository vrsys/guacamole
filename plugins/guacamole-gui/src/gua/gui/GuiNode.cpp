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

  ////////////////////////////////////////////////////////////////////////////////
  GuiNode::GuiNode(std::string const& name,
                   std::string const& resource_url,
                   math::mat4 const& transform)
    : GeometryNode(name, transform),
      resource_(nullptr),
      resource_url_(resource_url),
      resource_url_changed_(true)
  {}

  ////////////////////////////////////////////////////////////////////////////////

  void GuiNode::ray_test_impl(Ray const& ray, PickResult::Options options,
    Mask const& mask, std::set<PickResult>& hits) {

  }

  ////////////////////////////////////////////////////////////////////////////////

  void GuiNode::set_resource_url(std::string const& resource_url) {
    resource_url_ = resource_url;
    resource_url_changed_ = true;
  }

  ////////////////////////////////////////////////////////////////////////////////

  std::string const& GuiNode::get_resource_url() const {
    return resource_url_;
  }

  ////////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<GuiResource> const& GuiNode::get_resource() const {
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

  void GuiNode::update_cache() {

    if (resource_url_changed_) {
      if (resource_url_ != "") {
        if (!GeometryDatabase::instance()->is_supported(resource_url_)) {

          resource_ = std::make_shared<GuiResource>(resource_url_);
          GeometryDatabase::instance()->add(resource_url_, resource_);

        } else {

          resource_ = std::dynamic_pointer_cast<GuiResource>(GeometryDatabase::instance()->lookup(resource_url_));
        }

      }

      resource_url_changed_ = false;
    }

    GeometryNode::update_cache();
  }

  ////////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<node::Node> GuiNode::copy() const {
    auto result(std::make_shared<GuiNode>(get_name(), get_resource_url(), get_transform()));
    result->shadow_mode_ = shadow_mode_;
    result->resource_ = resource_;
    return result;
  }

}
