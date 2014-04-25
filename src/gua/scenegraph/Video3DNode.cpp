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
#include <gua/renderer/GeometryLoader.hpp> 
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
    Singleton<Video3DUberShader>::instance();
    bounding_box_ = math::BoundingBox<math::vec3>(math::vec3(-100.0,-100.0,-100.0),
                            math::vec3(100.0,100.0,100.0)); 
  }

  void Video3DNode::update_bounding_box() const {
    //Logger::LOG_WARNING << "not implemented: Video3DNode::update_bounding_box()" << std::endl;
  }

  void Video3DNode::ray_test_impl(RayNode const& ray, PickResult::Options options,
                             Mask const& mask, std::set<PickResult>& hits) {
    Logger::LOG_WARNING << "not implemented: Video3DNode::ray_test_impl()" << std::endl;
  }

  std::shared_ptr<Node> Video3DNode::copy() const {
    return std::make_shared<Video3DNode>(get_name(), filename_, material_, get_transform());
  }

}
