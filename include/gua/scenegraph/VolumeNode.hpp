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

#ifndef GUA_VOLUME_NODE_HPP
#define GUA_VOLUME_NODE_HPP

// guacamole headers
#include <gua/scenegraph/Node.hpp>
#include <gua/utils/configuration_macro.hpp>

// external headers
#include <string>

namespace gua {

/**
 * This class is used to represent a volume in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL VolumeNode : public Node {
  public:

    struct Configuration {
      GUA_ADD_PROPERTY(std::string, volume, "gua_volume_default");
    };

    Configuration data;

    VolumeNode() {};

    VolumeNode(std::string const& name,
               Configuration const& configuration = Configuration(),
               math::mat4 const& transform = math::mat4::identity());

    /*virtual*/ void accept(NodeVisitor&);

    /*virtual*/ void update_bounding_box() const;

    /*virtual*/ void ray_test_impl(RayNode const& ray,
                                   PickResult::Options options,
                                   Mask const& mask,
                                   std::set<PickResult>& hits);

  private:

    std::shared_ptr<Node> copy() const;
};

}

#endif  // GUA_VOLUME_NODE_HPP
