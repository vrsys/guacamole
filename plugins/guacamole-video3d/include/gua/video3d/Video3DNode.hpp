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
#include <gua/video3d/platform.hpp>
#include <gua/node/GeometryNode.hpp>
#include <gua/databases/GeometryDescription.hpp>

// external headers
#include <string>

/**
 * This class is used to represent kinect video in the SceneGraph.
 *
 */

namespace gua
{
class Video3DResource;

namespace node
{
class GUA_VIDEO3D_DLL Video3DNode : public GeometryNode
{
  public:
    Video3DNode(std::string const& name,
                std::string const& video_description = "gua_default_video",
                std::shared_ptr<Material> const& material = nullptr,
                math::mat4 const& transform = math::mat4::identity());

  public:
    void ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits) override;

    void update_cache() override;

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * \param visitor  A visitor to process the GeometryNode's data.
     */
    void accept(NodeVisitor& visitor) override;

    std::string const& get_video_description() const;
    void set_video_description(std::string const& v);
    void force_reload();

    std::shared_ptr<Material> const& get_material() const;
    void set_material(std::shared_ptr<Material> const& material);

  protected:
    std::shared_ptr<Node> copy() const override;

  private:
    std::shared_ptr<Video3DResource> video_;
    std::string video_description_;
    bool video_changed_;

    std::shared_ptr<Material> material_;
    bool material_changed_;
};

} // namespace node
} // namespace gua

#endif // GUA_VIDEO3D_NODE_HPP
