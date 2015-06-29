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

#ifndef GUA_CUBEMAP_NODE
#define GUA_CUBEMAP_NODE

#include <gua/platform.hpp>
#include <gua/node/SerializableNode.hpp>

namespace gua {
namespace node {

/**
 * This class is used to represent an ------- node in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL CubemapNode : public SerializableNode {
 public:

  /**
   * Constructor.
   *
   * This constructs an empty CubemapNode.
   *
   */
  CubemapNode() {};

  /**
   * Constructor.
   *
   * This constructs a CubemapNode with the given parameters.
   *
   * \param name           The name of the new CubemapNode.
   * \param transform      A matrix to describe the CubemapNode's
   *                       transformation.
   */
  CubemapNode(std::string const& name,
            math::mat4 const& transform = math::mat4::identity());

  /**
   * Accepts a visitor and calls concrete visit method.
   *
   * This method implements the visitor pattern for Nodes.
   *
   * \param visitor  A visitor to process the CubemapNode's data.
   */
  void accept(NodeVisitor& visitor) override;

  void set_texture_name(std::string const& name);
  std::string get_texture_name() const;

  float get_closest_distance() const;

 private:

  std::shared_ptr<Node> copy() const override;

  std::string texture_name_;
};

} // namespace node {
} // namespace gua {

#endif  // GUA_CUBEMAP_NODE
