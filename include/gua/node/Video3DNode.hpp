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
#include <gua/node/GeometryNode.hpp>

// external headers
#include <string>

/**
 * This class is used to represent kinect video in the SceneGraph.
 *
 */

namespace gua {

class GUA_DLL Video3DNode : public GeometryNode {
public:

    Video3DNode(std::string const& name,
                std::string const& filename = "gua_default_geometry",
                std::string const& material = "gua_default_material",
                math::mat4  const& transform = math::mat4::identity());

    void ray_test_impl(RayNode const& ray,
                       PickResult::Options options,
                       Mask const& mask,
                       std::set<PickResult>& hits) override;

    void update_cache() override;

protected:
    std::shared_ptr<Node> copy() const override;
};

}

#endif  // GUA_VIDEO3D_NODE_HPP
