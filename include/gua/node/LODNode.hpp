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

#ifndef GUA_LOD_NODE_HPP
#define GUA_LOD_NODE_HPP

#include <gua/platform.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/utils/configuration_macro.hpp>

namespace gua
{
namespace node
{
/**
 * This class is used to represent a level of detail node in the SceneGraph.
 *
 * While processing the LODNode, guacamole's renderer computes the distance
 * between the LODNode's translation and the current camera's translation in
 * world coordinates. The computed value is checked against a user-defined
 * distances vector. That vector contains distance values which are being mapped
 * to the LODNode's children sequentially. An entrance in the distances vector
 * therefore describes upto which distance between the current camera and the
 * LODNode the LODNode's child with the same index shall be visible.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL LODNode : public TransformNode
{
  public:
    struct Configuration
    {
        /**
         * A vector storing distances. Indices are mapped to the indices of the
         * LODNode's children vector.
         */
        GUA_ADD_PROPERTY(std::vector<float>, lod_distances, std::vector<float>());
    };

    /**
     * The LODNode's configuration.
     */
    Configuration data;

    /**
     * Constructor.
     *
     * This constructs an empty LODNode.
     *
     */
    LODNode() = default;

    /**
     * Constructor.
     *
     * This constructs a LODNode with the given parameters.
     *
     * \param name           The name of the new LODNode.
     * \param configuration  A configuration struct to define the LODNode's
     *                       properties.
     * \param transform      A matrix to describe the LODNode's
     *                       transformation.
     */
    LODNode(std::string const& name, Configuration const& configuration = Configuration(), math::mat4 const& transform = math::mat4::identity());

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * \param visitor  A visitor to process the LODNode's data.
     */
    void accept(NodeVisitor& visitor) override;

  private:
    std::shared_ptr<Node> copy() const;
};

} // namespace node
} // namespace gua

#endif // GUA_LOD_NODE_HPP
