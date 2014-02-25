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

#ifndef GUA_RAY_NODE_HPP
#define GUA_RAY_NODE_HPP

#include <gua/scenegraph/Node.hpp>
#include <gua/utils/configuration_macro.hpp>
#include <gua/utils/KDTree.hpp>

namespace gua {

/**
 * This class is used to represent a camera in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL RayNode : public Node {
 public:

  RayNode() {}

  /**
   * Constructor.
   *
   * This constructs a RayNode with the given parameters and calls
   * the constructor of base class Core with the type CAMERA.
   *
   * \param stereo_width  The gap between the eyes.
   */
  RayNode(std::string const& name,
          math::mat4 const& transform = math::mat4::identity());

  /**
   * Accepts a visitor and calls concrete visit method
   *
   * This method implements the visitor pattern for Nodes
   *
   */
  /* virtual */ void accept(NodeVisitor&);

  std::pair<float, float> intersect(
      math::BoundingBox<math::vec3> const& box) const;

  Ray const get_world_ray() const;

  void update_bounding_box() const;

  static const float END;

 private:

  /**
   *
   */
  std::shared_ptr<Node> copy() const;
};

}

#endif  // GUA_RAY_NODE_HPP
