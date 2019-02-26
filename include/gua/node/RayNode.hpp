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

#include <gua/node/Node.hpp>
#include <gua/utils/configuration_macro.hpp>
#include <gua/utils/KDTree.hpp>

namespace gua
{
namespace node
{
/**
 * This class is used to represent a ray in the SceneGraph.
 *
 * A RayNode is used to intersect scene geometry and utilized by
 * SceneGraph::ray_test().
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL RayNode : public Node
{
  public:
    /**
     * Constructor.
     *
     * This constructs an empty RayNode.
     *
     */
    RayNode() {}

    /**
     * Constructor.
     *
     * This constructs a RayNode with the given parameters.
     *
     * \param name           The name of the new RayNode.
     * \param transform      A matrix to describe the RayNode's transformation.
     */
    RayNode(std::string const& name, math::mat4 const& transform = math::mat4::identity());

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * \param visitor  A visitor to process the RayNode's data.
     */
    void accept(NodeVisitor& visitor) override;

    /**
     * Updates a RayNode's BoundingBox.
     *
     * The bounding box is updated according to the transformation matrices of
     * all children.
     */
    void update_bounding_box() const override;

    /**
     * Used internally to check whether a RayNode hits a given BoundingBox.
     *
     * \param box  The box to be checked against.
     *
     * \return std::pair<float, float>  A pair of parameters which describe the
     *                                  to potential intersecition points with a
     *                                  BoundingBox in ray space.
     */
    std::pair<float, float> intersect(math::BoundingBox<math::vec3> const& box) const;

    /**
     * Used internally to get a world-transformed mathematical representation of a
     * ray.
     *
     * \return Ray  A mathematical representation (origin, direction, length) of
     *              the RayNode.
     */
    Ray get_world_ray() const;

    /**
     * Used internally to check whether or not intersections occured.
     */
    static const float END;

  private:
    std::shared_ptr<Node> copy() const override;
};

} // namespace node
} // namespace gua

#endif // GUA_RAY_NODE_HPP
