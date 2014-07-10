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

#ifndef GUA_PBR_NODE_HPP
#define GUA_PBR_NODE_HPP

// guacamole headers
#include <gua/node/GeometryNode.hpp>

namespace gua {
namespace node {

/**
 * This class is used to represent pointcloud in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL PBRNode : public GeometryNode
{
public : // member

  PBRNode(std::string const& name,
          std::string const& filename = "gua_default_geometry",
          std::string const& material = "gua_default_material",
          math::mat4  const& transform = math::mat4::identity());

protected:

  std::shared_ptr<Node> copy() const override;

private : // attributes e.g. special attributes for drawing

};

}
}

#endif  // GUA_PBR_NODE_HPP
