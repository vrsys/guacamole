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

#ifndef GUA_GEOMETRY_NODE_HPP
#define GUA_GEOMETRY_NODE_HPP

// guacamole headers
#include <gua/scenegraph/Node.hpp>
#include <gua/utils/configuration_macro.hpp>

// external headers
#include <string>

namespace gua {

/**
 * This class is used to represent geometry in the SceneGraph.
 *
 * A GeometryNode only stores references to existing rendering assets stored in
 * guacamole's databases. GeometryNodes typically aren't instantiated directly
 * but by utilizing guacamole's GeometryLoader.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL GeometryNode : public Node {
  public:

    struct Configuration {
      GUA_ADD_PROPERTY(std::string,     geometry,   "gua_default_geometry");
      GUA_ADD_PROPERTY(std::string,     material,   "gua_default_material");
    };

    Configuration data;

    GeometryNode() {};

    /**
     * Constructor.
     *
     * This constructs a GeometryNode with the given parameters.
     *
     * \param name           The name of the new GeometryNode.
     * \param configuration  A configuration struct to define the GeometryNode's
     *                       properties.
     * \param transform      A matrix to describe the GeometryNode's
     *                       transformation.
     */
    GeometryNode(std::string const& name,
                 Configuration const& configuration = Configuration(),
                 math::mat4 const& transform = math::mat4::identity());

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * \param visitor  A visitor to process the GeometryNode's data.
     */
    /* virtual */ void accept(NodeVisitor& visitor);

    /**
     * Updates a GeometryNode's BoundingBox.
     *
     * The bounding box is updated according to the transformation matrices of
     * all children.
     */
    /*virtual*/ void update_bounding_box() const;

    /*virtual*/ void ray_test_impl(RayNode const& ray, PickResult::Options options,
                            Mask const& mask, std::set<PickResult>& hits);

  private:

    std::shared_ptr<Node> copy() const;
};

}

#endif  // GUA_GEOMETRY_NODE_HPP
