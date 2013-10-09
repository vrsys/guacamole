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

#ifndef GUA_SERIALIZED_NODE_HPP
#define GUA_SERIALIZED_NODE_HPP

// guacamole headers
#include <gua/math/math.hpp>

namespace gua {

/**
 * Stores information on a light for rendering.
 *
 * This is a struct used for serializing the graph.
 *
 * essentially the same as a std::pair<math::mat4, configuration_type>
 */
template <typename configuration_type> struct SerializedNode {

  SerializedNode() : transform(math::mat4::identity()), data() {}

  /**
   * Constructor.
   *
   * This creates a new serialized node.
   *
   * \param transform        The global transformation of this node.
   * \param color            The color of the light.
   */
  SerializedNode(math::mat4 const& t, configuration_type const& d)
      : transform(t), data(d) {}

  /**
   * The global transformation of this node.
   */
  math::mat4 transform;
  configuration_type data;
};

template <typename T>
inline SerializedNode<T> make_serialized_node(math::mat4 const& t, T const& d)
{
  return SerializedNode<T>(t, d);
}

}

#endif  // GUA_SERIALIZED_NODE_HPP
