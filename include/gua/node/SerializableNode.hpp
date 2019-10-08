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

#ifndef GUA_SERIALIZABLE_NODE_HPP
#define GUA_SERIALIZABLE_NODE_HPP

#include <gua/platform.hpp>
#include <gua/node/Node.hpp>

namespace gua
{
namespace node
{
/**
 * Nodes derived from this class will be stored in the serialized scene.
 *
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL SerializableNode : public Node
{
  public:
    /**
     * Constructor.
     *
     * This constructs an empty SerializableNode.
     *
     */
    SerializableNode(){};

    /**
     * Constructor.
     *
     * This constructs a SerializableNode with the given parameters.
     *
     * \param name           The name of the new SerializableNode.
     * \param transform      A matrix to describe the SerializableNode's
     *                       transformation.
     */
    SerializableNode(std::string const& name, math::mat4 const& transform = math::mat4::identity());
};

} // namespace node
} // namespace gua

#endif // GUA_SERIALIZABLE_NODE_HPP
