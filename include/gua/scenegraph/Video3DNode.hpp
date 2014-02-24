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

#ifndef GUA_VIDEO3D_NODE_HPP
#define GUA_VIDEO3D_NODE_HPP

// guacamole headers
#include <gua/scenegraph/Node.hpp>
#include <gua/utils/configuration_macro.hpp>

// external headers
#include <string>

/**
 * This class is used to represent geometry in the SceneGraph.
 *
 */

namespace gua {

class GUA_DLL Video3DNode : public Node {
  public:

    struct Configuration {
      GUA_ADD_PROPERTY(std::string,     video3d,   "gua_default_video3d");
      GUA_ADD_PROPERTY(std::string,     material,   "gua_default_material");
    };

    Configuration data;

    Video3DNode() {};

    /**
     * Constructor.
     *
     * This constructs a GeometryNode with the given parameters and calls
     * the constructor of base class Core with the type GEOMETRY.
     *
     * \param geometry  The name of the GeometryNode's geometry.
     * \param material  The name of the GeometryNodeCore's material.
     */
    Video3DNode(std::string const& name,
                 Configuration const& configuration = Configuration(),
                 math::mat4 const& transform = math::mat4::identity());

    /**
     * Accepts a visitor and calls concrete visit method
     *
     * This method implements the visitor pattern for Nodes
     *
     */
    /*virtual*/ void accept(NodeVisitor&);

    /*virtual*/ void update_bounding_box() const;

    /*virtual*/ void ray_test_impl(RayNode const& ray, PickResult::Options options,
                            Mask const& mask, std::set<PickResult>& hits);

  private:

    std::shared_ptr<Node> copy() const;
};

}

#endif  // GUA_VIDEO3D_NODE_HPP
